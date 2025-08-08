#include <print>
#include <memory>

#include <protocols/mbus/client.hpp>
#include <protocols/hw/client.hpp>

#include <helix/memory.hpp>

#include "controller.hpp"

std::vector<std::unique_ptr<Controller>> globalControllers;

async::detached bindPciController(mbus_ng::Entity entity) {
	protocols::hw::Device device((co_await entity.getRemoteLane()).unwrap());
	auto info = co_await device.getPciInfo();

	auto& ufsBarInfo = info.barInfo[0];
	assert(ufsBarInfo.ioType == protocols::hw::IoType::kIoTypeMemory);
	auto ufsBar = co_await device.accessBar(0);

	helix::UniqueDescriptor irq;

	if (info.numMsis) {
		irq = co_await device.installMsi(0);
		co_await device.enableMsi();
	} else {
		irq = co_await device.accessIrq();
	}

	co_await device.enableBusmaster();
	co_await device.enableBusIrq();
	co_await device.enableDma();

	helix::Mapping mapping{ufsBar, ufsBarInfo.offset, ufsBarInfo.length};

	auto controller = std::make_unique<Controller>(entity.id(), std::move(device),
			std::move(mapping), std::move(irq));
	controller->run();
	globalControllers.push_back(std::move(controller));
}

async::detached bindDtController(mbus_ng::Entity entity) {
	protocols::hw::Device device((co_await entity.getRemoteLane()).unwrap());

	auto info = co_await device.getDtInfo();
	auto reg = co_await device.accessDtRegister(0);

	helix::UniqueDescriptor irq;

	co_await device.enableBusIrq();
	irq = co_await device.installDtIrq(0);
	co_await device.enableDma();

	helix::Mapping mapping{reg, info.regs[0].offset, info.regs[0].length};

	auto controller = std::make_unique<Controller>(entity.id(), std::move(device),
			std::move(mapping), std::move(irq));
	controller->run();
	globalControllers.push_back(std::move(controller));
}

async::detached observeControllers() {
	auto filter = mbus_ng::Conjunction{{
		mbus_ng::EqualsFilter{"pci-class", "01"},
		mbus_ng::EqualsFilter{"pci-subclass", "09"},
		mbus_ng::Disjunction{{
			mbus_ng::EqualsFilter{"pci-interface", "00"},
			mbus_ng::EqualsFilter("pci-interface", "01")
		}}
	}};

	auto enumerator = mbus_ng::Instance::global().enumerate(filter);
	while (true) {
		auto [_, events] = (co_await enumerator.nextEvents()).unwrap();

		for (auto &event : events) {
			if (event.type != mbus_ng::EnumerationEvent::Type::created)
				continue;

			auto entity = co_await mbus_ng::Instance::global().getEntity(event.id);
			std::println(std::cout, "block/ufs: Detected controller");
			bindPciController(std::move(entity));
		}
	}
}

async::detached observeDtbControllers() {
	auto filter = mbus_ng::EqualsFilter{"dt.compatible=qcom,ufshc", ""};

	auto enumerator = mbus_ng::Instance::global().enumerate(filter);
	while (true) {
		auto [_, events] = (co_await enumerator.nextEvents()).unwrap();

		for (auto &event : events) {
			if (event.type != mbus_ng::EnumerationEvent::Type::created)
				continue;

			auto entity = co_await mbus_ng::Instance::global().getEntity(event.id);
			std::println(std::cout, "block/ufs: Detected dt controller");
			bindDtController(std::move(entity));
		}
	}
}

int main() {
	std::cout << "block/ufs: Starting driver\n";

	observeControllers();
	//observeDtbControllers();
	async::run_forever(helix::currentDispatcher);
}
