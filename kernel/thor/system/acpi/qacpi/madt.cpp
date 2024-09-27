#include <algorithm>
#include <frg/optional.hpp>
#include <frg/manual_box.hpp>
#include <frg/vector.hpp>
#include <eir/interface.hpp>
#include <thor-internal/arch/cpu.hpp>
#include <thor-internal/fiber.hpp>
#include <thor-internal/kernel_heap.hpp>
#include <thor-internal/main.hpp>
#include <thor-internal/acpi/acpi.hpp>
#include <thor-internal/acpi/pm-interface.hpp>
#include <thor-internal/pci/pci.hpp>

#include "madt.hpp"

namespace thor {
namespace acpi {

// Note: since firmware often provides unaligned MADTs,
//       we just mark all MADT structs as [[gnu::packed]].

struct [[gnu::packed]] MadtHeader {
	uint32_t localApicAddress;
	uint32_t flags;
};

struct [[gnu::packed]] MadtGenericEntry {
	uint8_t type;
	uint8_t length;
};

struct [[gnu::packed]] MadtLocalEntry {
	MadtGenericEntry generic;
	uint8_t processorId;
	uint8_t localApicId;
	uint32_t flags;
};

namespace local_flags {
	static constexpr uint32_t enabled = 1;
};

struct [[gnu::packed]] MadtIoEntry {
	MadtGenericEntry generic;
	uint8_t ioApicId;
	uint8_t reserved;
	uint32_t mmioAddress;
	uint32_t systemIntBase;
};

enum OverrideFlags {
	polarityMask = 0x03,
	polarityDefault = 0x00,
	polarityHigh = 0x01,
	polarityLow = 0x03,

	triggerMask = 0x0C,
	triggerDefault = 0x00,
	triggerEdge = 0x04,
	triggerLevel = 0x0C
};

struct [[gnu::packed]] MadtIntOverrideEntry {
	MadtGenericEntry generic;
	uint8_t bus;
	uint8_t sourceIrq;
	uint32_t systemInt;
	uint16_t flags;
};

struct [[gnu::packed]] MadtLocalNmiEntry {
	MadtGenericEntry generic;
	uint8_t processorId;
	uint16_t flags;
	uint8_t localInt;
};

frg::manual_box<qacpi::Context> globalCtx;
acpi_fadt *globalFadt;
static frg::manual_box<frg::vector<acpi_sdt_hdr *, KernelAlloc>> tables;

frg::optional<uint64_t> intFromPackageQacpi(qacpi::ObjectRef &pkg, size_t index) {
	auto obj = globalCtx->get_pkg_element(pkg, index);
	if(!obj)
		return frg::null_opt;

	if(!obj->get<uint64_t>())
		return frg::null_opt;

	return obj->get_unsafe<uint64_t>();
}

static void *acpiMap(uintptr_t phys, size_t size) {
	auto misalign = phys & 0xfff;
	phys &= ~0xfff;
	size = (size + misalign + 0xfff) & ~0xfff;
	auto ptr = KernelVirtualMemory::global().allocate(size);
	for(size_t pg = 0; pg < size; pg += kPageSize)
		KernelPageSpace::global().mapSingle4k((VirtualAddr)ptr + pg, phys + pg,
				page_access::write, CachingMode::null);

	return static_cast<char *>(ptr) + misalign;
}

static void acpiUnmap(void *ptr, size_t size) {
	auto misalign = reinterpret_cast<uintptr_t>(ptr) & 0xfff;
	ptr = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(ptr) & ~0xfff);
	size = (size + misalign + 0xfff) & ~0xfff;
	for(size_t pg = 0; pg < size; pg += kPageSize)
		KernelPageSpace::global().unmapSingle4k(reinterpret_cast<VirtualAddr>(ptr) + pg);

	KernelVirtualMemory::global().deallocate(ptr, size);
}

void *getTable(const char *signature, size_t index) {
	for(auto table : *tables) {
		if(memcmp(table->signature, signature, 4) == 0 && index-- == 0)
			return table;
	}

	return nullptr;
}

} } // namespace thor::acpi

// --------------------------------------------------------

namespace thor {

frg::manual_box<frg::optional<GlobalIrqInfo>> isaIrqOverrides[16];

GlobalIrqInfo resolveIsaIrq(unsigned int irq) {
	assert(irq < 16);
	if((*isaIrqOverrides[irq]))
		return *(*isaIrqOverrides[irq]);
	return GlobalIrqInfo{irq, IrqConfiguration{TriggerMode::edge, Polarity::high}};
}

// Same as resolveIsaIrq(irq) but allows to set more specific configuration options.
GlobalIrqInfo resolveIsaIrq(unsigned int irq, IrqConfiguration desired) {
	if(irq < 16 && *isaIrqOverrides[irq]) {
		assert(desired.compatible((*isaIrqOverrides[irq])->configuration));
		return *(*isaIrqOverrides[irq]);
	}
	return GlobalIrqInfo{irq, desired};
}

// --------------------------------------------------------

void configureIrq(GlobalIrqInfo info) {
#ifdef __x86_64__
	auto pin = getGlobalSystemIrq(info.gsi);
	assert(pin);
	pin->configure(info.configuration);
#endif
}

} // namespace thor

// --------------------------------------------------------

namespace thor {
namespace acpi {

void bootOtherProcessors() {
	acpi_sdt_hdr *madt = static_cast<acpi_sdt_hdr *>(getTable("APIC"));
	assert(madt);

	infoLogger() << "thor: Booting APs." << frg::endlog;

	size_t offset = sizeof(acpi_sdt_hdr) + sizeof(MadtHeader);
	while(offset < madt->length) {
		auto generic = (MadtGenericEntry *)(reinterpret_cast<char *>(madt) + offset);
		if(generic->type == 0) { // local APIC
			auto entry = (MadtLocalEntry *)generic;
			// TODO: Support BSPs with APIC ID != 0.
			if((entry->flags & local_flags::enabled)
					&& entry->localApicId) // We ignore the BSP here.
				bootSecondary(entry->localApicId);
		}
		offset += generic->length;
	}
}

// --------------------------------------------------------

void dumpMadt() {
	acpi_sdt_hdr *madt = static_cast<acpi_sdt_hdr *>(getTable("APIC"));
	assert(madt);

	infoLogger() << "thor: Dumping MADT" << frg::endlog;

	size_t offset = sizeof(acpi_sdt_hdr) + sizeof(MadtHeader);
	while(offset < madt->length) {
		auto generic = (MadtGenericEntry *)(reinterpret_cast<char *>(madt) + offset);
		if(generic->type == 0) { // local APIC
			auto entry = (MadtLocalEntry *)generic;
			infoLogger() << "    Local APIC id: "
					<< (int)entry->localApicId
					<< ((entry->flags & local_flags::enabled) ? "" :" (disabled)")
					<< frg::endlog;
		}else if(generic->type == 1) { // I/O APIC
			auto entry = (MadtIoEntry *)generic;
			infoLogger() << "    I/O APIC id: " << (int)entry->ioApicId
					<< ", sytem interrupt base: " << (int)entry->systemIntBase
					<< frg::endlog;
		}else if(generic->type == 2) { // interrupt source override
			auto entry = (MadtIntOverrideEntry *)generic;

			const char *bus, *polarity, *trigger;
			if(entry->bus == 0) {
				bus = "ISA";
			}else{
				panicLogger() << "Unexpected bus in MADT interrupt override"
						<< frg::endlog;
			}

			if((entry->flags & OverrideFlags::polarityMask) == OverrideFlags::polarityDefault) {
				polarity = "default";
			}else if((entry->flags & OverrideFlags::polarityMask) == OverrideFlags::polarityHigh) {
				polarity = "high";
			}else if((entry->flags & OverrideFlags::polarityMask) == OverrideFlags::polarityLow) {
				polarity = "low";
			}else{
				panicLogger() << "Unexpected polarity in MADT interrupt override"
						<< frg::endlog;
			}

			if((entry->flags & OverrideFlags::triggerMask) == OverrideFlags::triggerDefault) {
				trigger = "default";
			}else if((entry->flags & OverrideFlags::triggerMask) == OverrideFlags::triggerEdge) {
				trigger = "edge";
			}else if((entry->flags & OverrideFlags::triggerMask) == OverrideFlags::triggerLevel) {
				trigger = "level";
			}else{
				panicLogger() << "Unexpected trigger mode in MADT interrupt override"
						<< frg::endlog;
			}

			infoLogger() << "    Int override: " << bus << " IRQ " << (int)entry->sourceIrq
					<< " is mapped to GSI " << entry->systemInt
					<< " (Polarity: " << polarity << ", trigger mode: " << trigger
					<< ")" << frg::endlog;
		}else if(generic->type == 4) { // local APIC NMI source
			auto entry = (MadtLocalNmiEntry *)generic;
			infoLogger() << "    Local APIC NMI: processor " << (int)entry->processorId
					<< ", lint: " << (int)entry->localInt << frg::endlog;
		}else{
			infoLogger() << "    Unexpected MADT entry of type "
					<< generic->type << frg::endlog;
		}
		offset += generic->length;
	}
}

extern "C" EirInfo *thorBootInfoPtr;

initgraph::Stage *getTablesDiscoveredStage() {
	static initgraph::Stage s{&globalInitEngine, "acpi.tables-discovered"};
	return &s;
}

initgraph::Stage *getNsAvailableStage() {
	static initgraph::Stage s{&globalInitEngine, "acpi.ns-available"};
	return &s;
}

KernelFiber *acpiFiber;

static initgraph::Task initTablesTask{&globalInitEngine, "acpi.initialize",
	initgraph::Entails{getTablesDiscoveredStage()},
	[] {
		auto *rsdp = static_cast<acpi_rsdp *>(acpiMap(thorBootInfoPtr->acpiRsdp, sizeof(acpi_rsdp)));

		tables.initialize(*kernelAlloc);

		if(rsdp->revision >= 2) {
			auto *xsdtHdr = static_cast<acpi_xsdt *>(acpiMap(rsdp->xsdt_addr, sizeof(acpi_xsdt)));
			auto *xsdt = static_cast<acpi_xsdt *>(acpiMap(rsdp->xsdt_addr, xsdtHdr->hdr.length));

			uint32_t count = (xsdt->hdr.length - sizeof(acpi_xsdt)) / 8;
			for(uint32_t i = 0; i < count; ++i) {
				auto *hdr = static_cast<acpi_sdt_hdr *>(acpiMap(xsdt->entries[i], sizeof(acpi_sdt_hdr)));
				auto *table = static_cast<acpi_sdt_hdr *>(acpiMap(xsdt->entries[i], hdr->length));
				acpiUnmap(hdr, sizeof(acpi_sdt_hdr));
				tables->push(table);
			}

			acpiUnmap(xsdt, xsdtHdr->hdr.length);
			acpiUnmap(xsdtHdr, sizeof(acpi_xsdt));
		} else {
			auto *rsdtHdr = static_cast<acpi_rsdt *>(acpiMap(rsdp->rsdt_addr, sizeof(acpi_rsdt)));
			auto *rsdt = static_cast<acpi_rsdt *>(acpiMap(rsdp->rsdt_addr, rsdtHdr->hdr.length));

			uint32_t count = (rsdt->hdr.length - sizeof(acpi_rsdt)) / 4;
			for(uint32_t i = 0; i < count; ++i) {
				auto *hdr = static_cast<acpi_sdt_hdr *>(acpiMap(rsdt->entries[i], sizeof(acpi_sdt_hdr)));
				auto *table = static_cast<acpi_sdt_hdr *>(acpiMap(rsdt->entries[i], hdr->length));
				acpiUnmap(hdr, sizeof(acpi_sdt_hdr));
				tables->push(table);
			}
			
			acpiUnmap(rsdt, rsdtHdr->hdr.length);
			acpiUnmap(rsdtHdr, sizeof(acpi_rsdt));
		}

		acpiUnmap(rsdp, sizeof(acpi_rsdp));

		globalFadt = static_cast<acpi_fadt *>(getTable("FACP", 0));
		assert(globalFadt);
	}
};

static initgraph::Task discoverIoApicsTask{&globalInitEngine, "acpi.discover-ioapics",
	initgraph::Requires{getTablesDiscoveredStage(),
		getFibersAvailableStage()},
	initgraph::Entails{getTaskingAvailableStage()},
	[] {
		dumpMadt();

		acpi_sdt_hdr *madt = static_cast<acpi_sdt_hdr *>(getTable("APIC"));
		assert(madt);

		// Configure all interrupt controllers.
		// TODO: This should be done during thor's initialization in order to avoid races.
		infoLogger() << "thor: Configuring I/O APICs." << frg::endlog;

		size_t offset = sizeof(acpi_sdt_hdr) + sizeof(MadtHeader);
		while(offset < madt->length) {
			auto generic = (MadtGenericEntry *)(reinterpret_cast<char *>(madt) + offset);
			if(generic->type == 1) { // I/O APIC
				auto entry = (MadtIoEntry *)generic;
#ifdef __x86_64__
				setupIoApic(entry->ioApicId, entry->systemIntBase, entry->mmioAddress);
#endif
			}
			offset += generic->length;
		}

		// Determine IRQ override configuration.
		for(int i = 0; i < 16; i++)
			isaIrqOverrides[i].initialize();

		offset = sizeof(acpi_sdt_hdr) + sizeof(MadtHeader);
		while(offset < madt->length) {
			auto generic = (MadtGenericEntry *)(reinterpret_cast<char *>(madt) + offset);
			if(generic->type == 2) { // interrupt source override
				auto entry = (MadtIntOverrideEntry *)generic;

				// ACPI defines only ISA IRQ overrides.
				assert(entry->bus == 0);
				assert(entry->sourceIrq < 16);

				GlobalIrqInfo line;
				line.gsi = entry->systemInt;

				auto trigger = entry->flags & OverrideFlags::triggerMask;
				auto polarity = entry->flags & OverrideFlags::polarityMask;
				if(trigger == OverrideFlags::triggerDefault
						&& polarity == OverrideFlags::polarityDefault) {
					line.configuration.trigger = TriggerMode::edge;
					line.configuration.polarity = Polarity::high;
				}else{
					assert(trigger != OverrideFlags::triggerDefault);
					assert(polarity != OverrideFlags::polarityDefault);

					switch(trigger) {
					case OverrideFlags::triggerEdge:
						line.configuration.trigger = TriggerMode::edge; break;
					case OverrideFlags::triggerLevel:
						line.configuration.trigger = TriggerMode::level; break;
					default:
						panicLogger() << "Illegal IRQ trigger mode in MADT" << frg::endlog;
					}

					switch(polarity) {
					case OverrideFlags::polarityHigh:
						line.configuration.polarity = Polarity::high; break;
					case OverrideFlags::polarityLow:
						line.configuration.polarity = Polarity::low; break;
					default:
						panicLogger() << "Illegal IRQ polarity in MADT" << frg::endlog;
					}
				}

				assert(!(*isaIrqOverrides[entry->sourceIrq]));
				*isaIrqOverrides[entry->sourceIrq] = line;
			}
			offset += generic->length;
		}
	}
};

static initgraph::Task loadAcpiNamespaceTask{&globalInitEngine, "acpi.load-namespace",
	initgraph::Requires{getTaskingAvailableStage(),
		pci::getBus0AvailableStage()},
	initgraph::Entails{getNsAvailableStage()},
	[] {
		globalEventCtx.initialize();
		auto status = globalEventCtx->init(globalFadt);
		assert(status == qacpi::Status::Success);

		initGlue();

		auto *dsdtHdr = static_cast<acpi_dsdt *>(acpiMap(globalFadt->dsdt, sizeof(acpi_dsdt)));
		auto *dsdtPtr = static_cast<acpi_dsdt *>(acpiMap(globalFadt->dsdt, dsdtHdr->hdr.length));
		acpiUnmap(dsdtHdr, sizeof(acpi_dsdt));
		uint32_t dsdtAmlSize = dsdtPtr->hdr.length - sizeof(acpi_sdt_hdr);

		globalCtx.initialize(dsdtPtr->hdr.revision, qacpi::LogLevel::Info);

		if(auto status = globalCtx->init(); status != qacpi::Status::Success) {
			panicLogger() << "thor: qacpi failed to initialize: "
					<< qacpi::status_to_str(status) << frg::endlog;
		}

		if(auto status = globalCtx->load_table(dsdtPtr->definition_block, dsdtAmlSize);
				status != qacpi::Status::Success) {
			panicLogger() << "thor: qacpi failed to load dsdt: "
					<< qacpi::status_to_str(status) << frg::endlog;
		}

		for(size_t i = 0;; ++i) {
			auto *ssdtPtr = static_cast<acpi_ssdt *>(getTable("SSDT", i));
			if(!ssdtPtr)
				break;

			uint32_t ssdtAmlSize = ssdtPtr->hdr.length - sizeof(acpi_sdt_hdr);
			if(auto status = globalCtx->load_table(ssdtPtr->definition_block, ssdtAmlSize);
				status != qacpi::Status::Success) {
				panicLogger() << "thor: qacpi failed to load ssdt: "
						<< qacpi::status_to_str(status) << frg::endlog;
			}
		}

		auto ret = qacpi::ObjectRef::empty();
		qacpi::ObjectRef arg;
		assert(arg);
		// apic mode
		arg->data = uint64_t {1};
		status = globalCtx->evaluate("_PIC", ret, &arg, 1);
		if(status != qacpi::Status::Success && status != qacpi::Status::NotFound) {
			panicLogger() << "thor: qacpi failed to switch to IO-APIC mode: "
					<< qacpi::status_to_str(status) << frg::endlog;
		}

		initEc();

		if (auto status = globalCtx->init_namespace(); status != qacpi::Status::Success) {
			panicLogger() << "thor: qacpi failed to init namespace: "
					<< qacpi::status_to_str(status) << frg::endlog;
		}

		// Configure the ISA IRQs.
		// TODO: This is a hack. We assume that HPET will use legacy replacement.
		infoLogger() << "thor: Configuring ISA IRQs." << frg::endlog;
		configureIrq(resolveIsaIrq(0));
		configureIrq(resolveIsaIrq(1));
		configureIrq(resolveIsaIrq(4));
		configureIrq(resolveIsaIrq(12));
		configureIrq(resolveIsaIrq(14));

		initEvents();
	}
};

static initgraph::Task bootApsTask{&globalInitEngine, "acpi.boot-aps",
	initgraph::Requires{&loadAcpiNamespaceTask},
	[] {
		bootOtherProcessors();
	}
};

static initgraph::Task initPmInterfaceTask{&globalInitEngine, "acpi.init-pm-interface",
	initgraph::Requires{&loadAcpiNamespaceTask},
	[] {
		initializePmInterface();
	}
};

} } // namespace thor::acpi
