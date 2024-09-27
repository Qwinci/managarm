#include <thor-internal/arch/pic.hpp>
#include <thor-internal/fiber.hpp>
#include <thor-internal/io.hpp>
#include <thor-internal/kernel_heap.hpp>
#include <thor-internal/address-space.hpp>
#include <thor-internal/acpi/acpi.hpp>
#include <thor-internal/pci/pci.hpp>
#include <thor-internal/pci/pci_legacy.hpp>
#include <thor-internal/pci/pcie_ecam.hpp>
#include <thor-internal/main.hpp>
#include <system/acpi/qacpi/madt.hpp>

#include <qacpi/ns.hpp>
#include <qacpi/resources.hpp>

namespace thor {

struct AcpiPciRouting {
	uint32_t address;
	uint32_t pin;
	qacpi::NamespaceNode *source;
	uint32_t sourceIndex;
};

static qacpi::Status getPciRoutings(qacpi::NamespaceNode *node, frg::vector<AcpiPciRouting, KernelAlloc> &routings) {
	auto ret = qacpi::ObjectRef::empty();
	auto status = acpi::globalCtx->evaluate(node, "_PRT", ret);
	if(status != qacpi::Status::Success)
		return status;
	
	auto *pkg = ret->get<qacpi::Package>();
	assert(pkg);

	for(uint32_t i = 0; i < pkg->size(); ++i) {
		auto elem = acpi::globalCtx->get_pkg_element(ret, i);
		assert(elem);
		auto *elemPkg = elem->get<qacpi::Package>();
		assert(elemPkg);
		assert(elemPkg->size() == 4);

		auto addressElem = acpi::globalCtx->get_pkg_element(elem, 0);
		assert(addressElem);
		assert(addressElem->get<uint64_t>());
		uint64_t address = addressElem->get_unsafe<uint64_t>();

		auto pinElem = acpi::globalCtx->get_pkg_element(elem, 1);
		assert(pinElem);
		assert(pinElem->get<uint64_t>());
		uint64_t pin = pinElem->get_unsafe<uint64_t>();

		qacpi::NamespaceNode *source = nullptr;
		auto sourceObj = acpi::globalCtx->get_pkg_element(elem, 2);
		assert(sourceObj);
		if(sourceObj->get<qacpi::Device>()) {
			source = sourceObj->node;
		} else {
			assert(sourceObj->get<uint64_t>());
		}

		auto sourceIndexElem = acpi::globalCtx->get_pkg_element(elem, 3);
		assert(sourceIndexElem);
		assert(sourceIndexElem->get<uint64_t>());
		uint64_t sourceIndex = sourceIndexElem->get_unsafe<uint64_t>();

		routings.push({
			.address = static_cast<uint32_t>(address),
			.pin = static_cast<uint32_t>(pin),
			.source = source,
			.sourceIndex = static_cast<uint32_t>(sourceIndex)
		});
	}

	return qacpi::Status::Success;
}

} // namespace thor

namespace thor::pci {

struct AcpiPciIrqRouter : PciIrqRouter {
	AcpiPciIrqRouter(PciIrqRouter *parent_, PciBus *associatedBus_, qacpi::NamespaceNode *node);

	PciIrqRouter *makeDownstreamRouter(PciBus *bus) override;

private:
	qacpi::NamespaceNode *acpiNode = nullptr;
};

AcpiPciIrqRouter::AcpiPciIrqRouter(PciIrqRouter *parent_, PciBus *associatedBus_,
		qacpi::NamespaceNode *node)
: PciIrqRouter{parent_, associatedBus_}, acpiNode{node} {
	if(!acpiNode) {
		for(int i = 0; i < 4; i++) {
			bridgeIrqs[i] = parent->resolveIrqRoute(
					associatedBus->associatedBridge->slot, static_cast<IrqIndex>(i + 1));
			if(bridgeIrqs[i])
				infoLogger() << "thor:     Bridge IRQ [" << i << "]: "
						<< bridgeIrqs[i]->name() << frg::endlog;
		}

		routingModel = RoutingModel::expansionBridge;
		return;
	}

	frg::vector<AcpiPciRouting, KernelAlloc> pciRoutings{*kernelAlloc};

	auto ret = getPciRoutings(acpiNode, pciRoutings);
	if(ret == qacpi::Status::NotFound) {
		if(parent) {
			infoLogger() << "thor: There is no _PRT for bus " << associatedBus->busId << ";"
					" assuming expansion bridge routing" << frg::endlog;
			for(int i = 0; i < 4; i++) {
				bridgeIrqs[i] = parent->resolveIrqRoute(
						associatedBus->associatedBridge->slot, static_cast<IrqIndex>(i + 1));
				if(bridgeIrqs[i])
					infoLogger() << "thor:     Bridge IRQ [" << i << "]: "
							<< bridgeIrqs[i]->name() << frg::endlog;
			}

			routingModel = RoutingModel::expansionBridge;
		}else{
			infoLogger() << "thor: There is no _PRT for bus " << associatedBus->busId << ";"
					" giving up IRQ routing of this bus" << frg::endlog;
		}
		return;
	} else if (ret != qacpi::Status::Success) {
		infoLogger() << "thor: Failed to evaluate _PRT: "
					<< qacpi::status_to_str(ret) << frg::endlog;

		auto bus = node->absolute_path();
		frg::string_view busView{bus.data(), bus.size()};
		infoLogger() << "giving up IRQ routing of bus: " << busView << frg::endlog;
		return;
	}

	// Walk through the PRT and determine the routing.
	for (const auto &entry : pciRoutings) {
		// These are the defaults
		auto triggering = TriggerMode::level;
		auto polarity = Polarity::low;
		auto gsi = entry.sourceIndex;
		auto slot = (entry.address >> 16) & 0xFFFF;

		assert((entry.address & 0xFFFF) == 0xFFFF && "TODO: support routing of individual functions");
		if(entry.source) {
			auto crsObj = qacpi::ObjectRef::empty();
			auto status = acpi::globalCtx->evaluate(entry.source, "_CRS", crsObj);
			assert(status == qacpi::Status::Success);
			auto crsBuffer = crsObj->get<qacpi::Buffer>();
			assert(crsBuffer);

			size_t offset = 0;
			uint32_t index = 0;
			bool found = false;
			while(true) {
				qacpi::Resource res;
				status = qacpi::resource_parse(
					crsBuffer->data(), crsBuffer->size(),
					offset, res);
				if(status == qacpi::Status::EndOfResources)
					break;
				else
				 	assert(status == qacpi::Status::Success);

				if(index != entry.sourceIndex) {
					++index;
					continue;
				}

				found = true;

				if(auto irq = res.get<qacpi::IrqDescriptor>()) {
					if(irq->info & qacpi::IRQ_INFO_EDGE_TRIGGERED)
						triggering = TriggerMode::edge;
					if(!(irq->info & qacpi::IRQ_INFO_ACTIVE_LOW))
						polarity = Polarity::high;
					for(uint32_t i = 0; i < 16; ++i) {
						if(irq->mask_bits & (1 << i)) {
							gsi = i;
							break;
						}
					}
					break;
				} else if(auto extIrq = res.get<qacpi::ExtendedIrqDescriptor>()) {
					if(extIrq->info & qacpi::EXT_IRQ_INFO_EDGE_TRIGGERED)
						triggering = TriggerMode::edge;
					if(!(extIrq->info & qacpi::EXT_IRQ_INFO_ACTIVE_LOW))
						polarity = Polarity::high;
					assert(extIrq->irq_table_length);
					gsi = extIrq->irq_table[0];
					break;
				} else {
					assert(!"unsupported acpi irq descriptor");
				}
			}

			assert(found);
		}

		auto index = static_cast<IrqIndex>(entry.pin + 1);

		infoLogger() << "    Route for slot " << slot
				<< ", " << nameOf(index) << ": "
				<< "GSI " << gsi << frg::endlog;

		configureIrq(GlobalIrqInfo{gsi, { triggering, polarity}});
		auto pin = getGlobalSystemIrq(gsi);
		routingTable.push({slot, index, pin});
	}

	routingModel = RoutingModel::rootTable;
}

PciIrqRouter *AcpiPciIrqRouter::makeDownstreamRouter(PciBus *bus) {
	qacpi::NamespaceNode *deviceHandle = nullptr;

	if (acpiNode) {
		uint64_t targetAddr = (bus->associatedBridge->slot << 16) | bus->associatedBridge->function;

		frg::vector<qacpi::NamespaceNode *, KernelAlloc> stack{*kernelAlloc};
		stack.push(acpiNode);
		while(!stack.empty()) {
			auto node = stack.back();
			stack.pop();

			auto obj = node->get_object();
			if(!obj || !obj->is_device())
				continue;

			auto addrObj = qacpi::ObjectRef::empty();
			auto ret = acpi::globalCtx->evaluate(node, "_ADR", addrObj);
			if(ret != qacpi::Status::Success && ret != qacpi::Status::NotFound)
				continue;

			if(ret == qacpi::Status::Success) {
				assert(addrObj->get<uint64_t>());
				if(addrObj->get_unsafe<uint64_t>() == targetAddr) {
					deviceHandle = node;
					break;
				}
			}else {
				if(targetAddr == 0) {
					deviceHandle = node;
					break;
				}
			}
		}
	}

	if (deviceHandle) {
		auto acpiPath = deviceHandle->absolute_path();
		infoLogger() << "            ACPI: " << frg::string_view{acpiPath.data(), acpiPath.size()} << frg::endlog;
	}

	return frg::construct<AcpiPciIrqRouter>(*kernelAlloc, this, bus, deviceHandle);
}

static void addLegacyConfigIo() {
	auto io = frg::construct<LegacyPciConfigIo>(*kernelAlloc);
	for (int i = 0; i < 256; i++) {
		addConfigSpaceIo(0, i, io);
	}
}

struct [[gnu::packed]] McfgEntry {
	uint64_t mmioBase;
	uint16_t segment;
	uint8_t busStart;
	uint8_t busEnd;
	uint32_t reserved;
};

static initgraph::Task discoverConfigIoSpaces{&globalInitEngine, "pci.discover-acpi-config-io",
	initgraph::Requires{acpi::getTablesDiscoveredStage()},
	initgraph::Entails{getBus0AvailableStage()},
	[] {
		auto *mcfgTbl = static_cast<acpi_sdt_hdr *>(acpi::getTable("MCFG"));
		if(!mcfgTbl) {
			urgentLogger() << "thor: No MCFG table!" << frg::endlog;
			addLegacyConfigIo();
			return;
		}

		if(mcfgTbl->length < sizeof(acpi_sdt_hdr) + 8 + sizeof(McfgEntry)) {
			urgentLogger() << "thor: MCFG table has no entries, assuming legacy PCI!"
					<< frg::endlog;
			addLegacyConfigIo();
			return;
		}

		size_t nEntries = (mcfgTbl->length - 44) / 16;
		auto mcfgEntries = (McfgEntry *)((uintptr_t)mcfgTbl + sizeof(acpi_sdt_hdr) + 8);
		for (size_t i = 0; i < nEntries; i++) {
			auto &entry = mcfgEntries[i];
			infoLogger() << "Found config space for segment " << entry.segment
				<< ", buses " << entry.busStart << "-" << entry.busEnd
				<< ", ECAM MMIO base at " << (void *)entry.mmioBase << frg::endlog;

			auto io = frg::construct<EcamPcieConfigIo>(*kernelAlloc,
					entry.mmioBase, entry.segment,
					entry.busStart, entry.busEnd);

			for (int j = entry.busStart; j <= entry.busEnd; j++) {
				addConfigSpaceIo(entry.segment, j, io);
			}
		}
	}
};

static initgraph::Task discoverAcpiRootBuses{&globalInitEngine, "pci.discover-acpi-root-buses",
	initgraph::Requires{getTaskingAvailableStage(), acpi::getNsAvailableStage()},
	initgraph::Entails{getDevicesEnumeratedStage()},
	[] {
		static constexpr qacpi::EisaId pciRootIds[] {
			qacpi::PCI_ID,
			qacpi::PCIE_ID
		};

		auto sb = acpi::globalCtx->get_root()->get_child("_SB_");
		acpi::globalCtx->discover_nodes(
			sb,
			pciRootIds, 2,
			[](qacpi::Context &ctx, qacpi::NamespaceNode *node) {
				uint64_t seg = 0;
				uint64_t bus = 0;

				auto segObj = qacpi::ObjectRef::empty();
				auto status = ctx.evaluate(node, "_SEG", segObj);
				if(status == qacpi::Status::Success) {
					assert(segObj->get<uint64_t>());
					seg = segObj->get_unsafe<uint64_t>();
				} else {
					assert(status == qacpi::Status::NotFound);
				}

				auto busObj = qacpi::ObjectRef::empty();
				status = ctx.evaluate(node, "_BBN", busObj);
				if(status == qacpi::Status::Success) {
					assert(busObj->get<uint64_t>());
					bus = busObj->get_unsafe<uint64_t>();
				} else {
					assert(status == qacpi::Status::NotFound);
				}

				infoLogger() << "thor: Found PCI host bridge " << frg::hex_fmt{seg} << ":"
					<< frg::hex_fmt{bus} << frg::endlog;

				PciMsiController *msiController = nullptr;
				#ifdef __x86_64__
					struct ApicMsiController final : PciMsiController {
						MsiPin *allocateMsiPin(frg::string<KernelAlloc> name) override {
							return allocateApicMsi(std::move(name));
						}
					};

					msiController = frg::construct<ApicMsiController>(*kernelAlloc);
				#endif

				auto rootBus = frg::construct<PciBus>(*kernelAlloc, nullptr, nullptr,
						getConfigIoFor(seg, bus), msiController, seg, bus);
				rootBus->irqRouter = frg::construct<AcpiPciIrqRouter>(*kernelAlloc, nullptr, rootBus, node);
				addRootBus(rootBus);
				return qacpi::IterDecision::Continue;
			});

		infoLogger() << "thor: Discovering PCI devices" << frg::endlog;
		enumerateAll();
	}
};

} // namespace thor::pci
