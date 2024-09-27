#include <thor-internal/main.hpp>
#include <thor-internal/acpi/acpi.hpp>
#include <thor-internal/debug.hpp>
#include <thor-internal/arch/ints.hpp>
#include <stdlib.h>

#include <frg/optional.hpp>

#include <qacpi/event.hpp>
#include <qacpi/utils.hpp>
#include <qacpi/resources.hpp>
#include <qacpi/ns.hpp>

#include "madt.hpp"

namespace thor::acpi {

static void regWrite(qacpi::Address &gas, uint8_t value) {
	auto ret = qacpi::write_to_addr(gas, value);
	assert(ret == qacpi::Status::Success);
}

uint8_t regRead(qacpi::Address &gas) {
	uint64_t reg;

	auto ret = qacpi::read_from_addr(gas, reg);
	assert(ret == qacpi::Status::Success);

	return reg;
}

void waitForBit(qacpi::Address &gas, uint8_t bit, bool value) {
	uint8_t reg;

	do {
		reg = regRead(gas);
	} while ((reg & bit) != value);
}

#define EC_OBF (1 << 0)
#define EC_IBF (1 << 1)
#define EC_BURST (1 << 4)
#define EC_SCI_EVT (1 << 5)

#define RD_EC 0x80
#define WR_EC 0x81
#define BE_EC 0x82
#define BD_EC 0x83
#define QR_EC 0x84

#define BURST_ACK 0x90

struct ECDevice {
	qacpi::NamespaceNode *node;
	frg::optional<uint16_t> gpeIdx;
	bool initialized = false;
	IrqSpinlock lock;

	qacpi::Address control;
	qacpi::Address data;

	void pollIbf() {
		waitForBit(control, EC_IBF, false);
	}

	void pollObf() {
		waitForBit(control, EC_OBF, true);
	}

	void writeOne(qacpi::Address &reg, uint8_t value) {
		pollIbf();
		regWrite(reg, value);
	}

	uint8_t readOne(qacpi::Address &reg) {
		pollObf();
		return regRead(reg);
	}

	void burstEnable() {
		writeOne(control, BE_EC);
		auto ec_ret = readOne(data);
		assert(ec_ret == BURST_ACK);
	}

	void burstDisable() {
		writeOne(control, BD_EC);
		waitForBit(control, EC_BURST, false);
	}

	uint8_t read(uint8_t offset) {
		writeOne(control, RD_EC);
		writeOne(data, offset);
		return readOne(data);
	}

	void write(uint8_t offset, uint8_t value) {
		writeOne(control, WR_EC);
		writeOne(data, offset);
		writeOne(data, value);
	}

	bool checkEvent(uint8_t &idx) {
		auto status = regRead(control);

		// We get an extra EC event when disabling burst, that's ok.
		if(!(status & EC_SCI_EVT))
			return false;

		burstEnable();
		writeOne(control, QR_EC);
		idx = readOne(data);
		burstDisable();

		return true;
	}
};
static frg::manual_box<ECDevice> ecDevice;

static qacpi::RegionSpaceHandler ecHandler{
	.attach = [](qacpi::Context *, qacpi::NamespaceNode *) {
		return qacpi::Status::Success;
	},
	.detach = [](qacpi::Context *, qacpi::NamespaceNode *) {
		return qacpi::Status::Success;
	},
	.read = [](qacpi::NamespaceNode *, uint64_t offset, uint8_t size, uint64_t &res, void *arg) {
		auto *ecDevice = static_cast<ECDevice *>(arg);

		if(size != 1) {
			infoLogger() << "thor: invalid EC access width " << size << frg::endlog;
			return qacpi::Status::InvalidArgs;
		}

		auto guard = frg::guard(&ecDevice->lock);

		ecDevice->burstEnable();

		res = ecDevice->read(offset);

		ecDevice->burstDisable();
		return qacpi::Status::Success;
	},
	.write = [](qacpi::NamespaceNode *, uint64_t offset, uint8_t size, uint64_t value, void *arg) {
		auto *ecDevice = static_cast<ECDevice *>(arg);

		if(size != 1) {
			infoLogger() << "thor: invalid EC access width " << size << frg::endlog;
			return qacpi::Status::InvalidArgs;
		}

		auto guard = frg::guard(&ecDevice->lock);

		ecDevice->burstEnable();

		ecDevice->write(offset, value);

		ecDevice->burstDisable();
		return qacpi::Status::Success;
	},
	.id = qacpi::RegionSpace::EmbeddedControl
};

void handleEcEvent(void *arg) {
	auto *ecDevice = static_cast<ECDevice *>(arg);

	auto guard = frg::guard(&ecDevice->lock);

	uint8_t idx;
	if(!ecDevice->checkEvent(idx))
		return;

	if(idx == 0) {
		infoLogger() << "thor: EC indicates no outstanding events" << frg::endlog;
		return;
	}

	static const char *hexChars = "0123456789ABCDEF";

	char methodName[5] = { '_', 'Q', hexChars[(idx >> 4) & 0xF], hexChars[idx & 0xF] };

	infoLogger() << "thor: evaluating EC query " << methodName << frg::endlog;

	auto res = qacpi::ObjectRef::empty();
	globalCtx->evaluate(ecDevice->node, methodName, res);
}

static bool initFromEcdt() {
	auto *ecdt = static_cast<acpi_ecdt *>(getTable("ECDT"));
	if(!ecdt) {
		infoLogger() << "thor: no ECDT detected" << frg::endlog;
		return false;
	}

	infoLogger() << "thor: found ECDT, EC@" << ecdt->ec_id << frg::endlog;

	auto *ecNode = globalCtx->find_node(nullptr, ecdt->ec_id);
	if(!ecNode) {
		infoLogger() << "thor: invalid EC path " << ecdt->ec_id << frg::endlog;
		return false;
	}

	ecDevice.initialize();
	ecDevice->node = ecNode;
	memcpy(&ecDevice->control, &ecdt->ec_control, sizeof(acpi_gas));
	memcpy(&ecDevice->data, &ecdt->ec_data, sizeof(acpi_gas));
	return true;
}

static void initFromNamespace() {
	qacpi::EisaId ec_id{"PNP0C09"};

	globalCtx->discover_nodes(nullptr, &ec_id, 1, [](qacpi::Context &, qacpi::NamespaceNode *node) {
		auto crsObj = qacpi::ObjectRef::empty();
		auto status = globalCtx->evaluate(node, "_CRS", crsObj);
		if(status != qacpi::Status::Success)
			return qacpi::IterDecision::Continue;

		auto crsBuffer = crsObj->get<qacpi::Buffer>();
		assert(crsBuffer);

		qacpi::Address dataReg{};
		dataReg.space_id = qacpi::RegionSpace::SystemIo;
		qacpi::Address controlReg{};
		controlReg.space_id = qacpi::RegionSpace::SystemIo;

		int index = 0;
		size_t offset = 0;
		while(true) {
			qacpi::Resource res{};
			status = qacpi::resource_parse(
				crsBuffer->data(), crsBuffer->size(),
				offset, res);
			if(status == qacpi::Status::EndOfResources)
				break;

			auto &reg = index == 0 ? dataReg : controlReg;

			if(auto io = res.get<qacpi::IoPortDescriptor>()) {
				reg.address = io->min_base;
				reg.reg_bit_width = io->length * 8;
				++index;
				if(index == 2)
					break;
			} else if(auto fixedIo = res.get<qacpi::FixedIoPortDescriptor>()) {
				reg.address = fixedIo->base;
				reg.reg_bit_width = fixedIo->length * 8;
				++index;
				if(index == 2)
					break;
			}
		}

		if(index != 2) {
			infoLogger() << "thor: didn't find all needed resources for EC" << frg::endlog;
			return qacpi::IterDecision::Continue;
		}

		ecDevice.initialize();
		ecDevice->node = node;
		ecDevice->control = controlReg;
		ecDevice->data = dataReg;

		auto fullPath = node->absolute_path();
		frg::string_view pathView{fullPath.data(), fullPath.size()};
		infoLogger() << "thor: found an EC@" << pathView << frg::endlog;

		return qacpi::IterDecision::Break;
	});
}

frg::manual_box<qacpi::events::Context> globalEventCtx{};

static void installEcHandlers() {
	auto valueObj = qacpi::ObjectRef::empty();
	globalCtx->evaluate(ecDevice->node, "_GLK", valueObj);

	uint64_t value = 0;
	if(valueObj && valueObj->get<uint64_t>()) {
		value = valueObj->get_unsafe<uint64_t>();
	}

	if(value)
		infoLogger() << "thor: EC requires locking (this is a TODO)" << frg::endlog;

	auto gpeObj = qacpi::ObjectRef::empty();
	auto status = globalCtx->evaluate(ecDevice->node, "_GPE", gpeObj);
	if (status != qacpi::Status::Success) {
		infoLogger() << "thor: EC has no associated _GPE" << frg::endlog;
		return;
	}
	assert(gpeObj->get<uint64_t>());

	ecDevice->gpeIdx = gpeObj->get_unsafe<uint64_t>();
	status = globalEventCtx->enable_gpe(
		*ecDevice->gpeIdx, qacpi::events::GpeTrigger::Edge,
		handleEcEvent, ecDevice.get()
	);
	assert(status == qacpi::Status::Success);

	ecDevice->initialized = true;

	ecHandler.arg = ecDevice.get();
	globalCtx->register_address_space_handler(&ecHandler);
}

void initEc() {
	bool earlyReg = true;

	if(!initFromEcdt()) {
		earlyReg = false;
		initFromNamespace();
	}

	if(!ecDevice) {
		infoLogger() << "thor: no EC devices on the system" << frg::endlog;
		return;
	}

	/*
	 * Don't attempt to run _REG early if firmware didn't explicitly ask for
	 * it in the form of providing an ECDT table. It might rely on the namespace
	 * being fully initialized in the _REG method(s).
	 */
	if (earlyReg)
		installEcHandlers();
}

static void asyncShutdown() {
	infoLogger() << "thor: shutting down..." << frg::endlog;

	auto status = globalEventCtx->prepare_for_sleep_state(*globalCtx, qacpi::events::SleepState::S5);
	assert(status == qacpi::Status::Success);

	disableInts();

	status = globalEventCtx->enter_sleep_state(qacpi::events::SleepState::S5);
	assert(status == qacpi::Status::Success);
}

static void handlePowerButton(void *) {
	asyncShutdown();
}

static void handlePowerButtonNotify(void *, qacpi::NamespaceNode *, uint64_t value) {
	// 0x80: S0 Power Button Pressed
	if (value != 0x80) {
		infoLogger() << "thor: ignoring unknown power button notify value " << value
					<< frg::endlog;
		return;
	}

	infoLogger() << "thor: shutting down because of power button notification"
				<< frg::endlog;

	asyncShutdown();
}

void initEvents() {
	/*
	 * We don't have any sort of power management subsystem,
	 * so just enable all GPEs that have an AML handler.
	 */
	auto status = globalEventCtx->enable_events_from_ns(*globalCtx);
	assert(status == qacpi::Status::Success);

	if(ecDevice) {
		if(!ecDevice->initialized)
			installEcHandlers();

		if(ecDevice->gpeIdx) {
			infoLogger() << "thor: enabling EC GPE " << *ecDevice->gpeIdx << frg::endlog;

			status = globalEventCtx->enable_gpe(
				*ecDevice->gpeIdx, qacpi::events::GpeTrigger::Edge,
				handleEcEvent, ecDevice.get()
			);
			assert(status == qacpi::Status::Success);
		}
	}

	status = globalEventCtx->enable_fixed_event(
		qacpi::events::FixedEvent::PowerButton,
		handlePowerButton, nullptr);
	assert(status == qacpi::Status::Success || status == qacpi::Status::Unsupported);

	/*
	 * Modern hardware uses power button devices instead of the fixed event.
	 * Search for them here and hook AML notifications.
	 */
	qacpi::EisaId powerButtonId{"PNP0C0C"};
	globalCtx->discover_nodes(nullptr, &powerButtonId, 1, [](qacpi::Context &, qacpi::NamespaceNode *node) {
		globalEventCtx->install_notify_handler(node, handlePowerButtonNotify, nullptr);
		return qacpi::IterDecision::Continue;
	});
}

}
