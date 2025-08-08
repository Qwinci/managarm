#include "controller.hpp"

#include <arch/variable.hpp>
#include <helix/timer.hpp>

#include <print>
#include <bit>

namespace {
	constexpr bool logCommands = false;
}

namespace regs {
	constexpr arch::bit_register<uint32_t> cap{0x0};
	constexpr arch::bit_register<uint32_t> version{0x8};
	constexpr arch::bit_register<uint32_t> interruptStatus{0x20};
	constexpr arch::bit_register<uint32_t> interruptEnable{0x24};
	constexpr arch::bit_register<uint32_t> hcs{0x30};
	constexpr arch::bit_register<uint32_t> hce{0x34};
	constexpr arch::bit_register<uint32_t> utriacr{0x4c};

	constexpr arch::scalar_register<uint32_t> transferListBaseLow{0x50};
	constexpr arch::scalar_register<uint32_t> transferListBaseHigh{0x54};
	constexpr arch::scalar_register<uint32_t> transferListDoorbell{0x58};
	constexpr arch::scalar_register<uint32_t> transferListClear{0x5c};
	constexpr arch::scalar_register<uint32_t> transferListRun{0x60};
	constexpr arch::scalar_register<uint32_t> transferListCompletion{0x64};

	constexpr arch::scalar_register<uint32_t> taskRequestListLow{0x70};
	constexpr arch::scalar_register<uint32_t> taskRequestListHigh{0x74};
	constexpr arch::scalar_register<uint32_t> taskRequestListDoorbell{0x78};
	constexpr arch::scalar_register<uint32_t> taskRequestListClear{0x7c};
	constexpr arch::scalar_register<uint32_t> taskRequestListRun{0x80};

	constexpr arch::bit_register<uint32_t> uicCmd{0x90};
	constexpr arch::scalar_register<uint32_t> uicCmdArg1{0x94};
	constexpr arch::scalar_register<uint32_t> uicCmdArg2{0x98};
	constexpr arch::scalar_register<uint32_t> uicCmdArg3{0x9c};
} // namespace regs

namespace cap {
	constexpr arch::field<uint32_t, uint8_t> numTransferSlots{0, 5};
	constexpr arch::field<uint32_t, uint8_t> numTaskSlots{18, 3};
	constexpr arch::field<uint32_t, bool> as64{24, 1};
} // namespace cap

namespace irqStatus {
	constexpr arch::field<uint32_t, bool> utrcs{0, 1};
	constexpr arch::field<uint32_t, bool> uccs{10, 1};
} // namespace irqStatus

namespace irqEnable {
	constexpr arch::field<uint32_t, bool> utrce{0, 1};
} // namespace irqEnable

namespace hcs {
	constexpr arch::field<uint32_t, bool> dp{0, 1};
	constexpr arch::field<uint32_t, bool> ucrdy{3, 1};
} // namespace hcs

namespace hce {
	constexpr arch::field<uint32_t, bool> hce{0, 1};
	constexpr arch::field<uint32_t, bool> cge{1, 1};
} // namespace hce

namespace utriacr {
	constexpr arch::field<uint32_t, uint8_t> iatoval{0, 8};
	constexpr arch::field<uint32_t, uint8_t> iacth{8, 5};
	constexpr arch::field<uint32_t, bool> ctr{16, 1};
	constexpr arch::field<uint32_t, bool> iasb{20, 1};
	constexpr arch::field<uint32_t, bool> iapwen{24, 1};
	constexpr arch::field<uint32_t, bool> iaen{31, 1};
} // namespace utriacr

enum class UicCommand : uint8_t {
	dmeGet = 0x1,
	dmeSet = 0x2,
	dmePeerGet = 0x3,
	dmePeerSet = 0x4,
	dmePowerOn = 0x10,
	dmePowerOff = 0x11,
	dmeEnable = 0x12,
	dmeReset = 0x14,
	dmeEndpointReset = 0x15,
	dmeLinkStartup = 0x16
};

namespace uicCmd {
	constexpr arch::field<uint32_t, UicCommand> opcode{0, 8};
} // namespace uicCmd

Controller::Controller(int64_t parentId, protocols::hw::Device hwDevice, helix::Mapping ufsRegs, helix::UniqueDescriptor irq)
	: hwDevice_{std::move(hwDevice)}, regsMapping_{std::move(ufsRegs)},
	regs_{regsMapping_.get()}, irq_{std::move(irq)},
	dmaPool_{}, taskRequestList_{&dmaPool_}, transferRequestList_{&dmaPool_},
	transferCompletions_{}, logicalUnits_{}, reportLunsUnit_{this, scsi::WELL_KNOWN_REPORT_LUNS_LUN | 1 << 7},
	parentId_{parentId}, irqSequence_{}, numTaskSlots_{}, numTransferSlots_{},
	transfersInProgress_{}, transferSlotsInUse_{}
{
}

async::result<int> Controller::findFreeTransferSlot_() {
	while (transfersInProgress_ >= numTransferSlots_) {
		co_await freeTransferSlotDoorbell_.async_wait();
	}

	for (int i = 0; i < numTransferSlots_; ++i) {
		if (!(transferSlotsInUse_ & 1U << i)) {
			co_return i;
		}
	}

	assert(!"block/ufs: No free slot found");
}

async::result<void> Controller::sendTransfer_(const TransferInfo& info, Completion &completion) {
	assert(info.ucdBase % 128 == 0);

	// 64-bit aligned
	assert(info.responseUpiuOffsetDwords % 2 == 0);
	assert(info.prdtOffsetDwords % 2 == 0);

	TransferRequest req{
		.flags = transferFlags::commandType(CMD_TYPE_UFS_STORAGE)
			| transferFlags::dir(info.dataDir)
			| transferFlags::interrupt(true),
		.dataUnitNumLower = 0,
		.status = transferStatus::invalidOcsValue,
		.dataUnitNumUpper = 0,
		.ucdBaseLower = static_cast<uint32_t>(info.ucdBase),
		.ucdBaseUpper = static_cast<uint32_t>(info.ucdBase >> 32),
		.responseUpiuLength = info.responseUpiuDwords,
		.responseUpiuOffset = info.responseUpiuOffsetDwords,
		.prdtLength = info.physicalRegionDescCount,
		.prdtOffset = info.prdtOffsetDwords
	};

	int slot = co_await findFreeTransferSlot_();

	transferCompletions_[slot] = &completion;
	transferRequestList_->requests[slot] = req;
	transferSlotsInUse_ |= 1U << slot;
	regs_.store(regs::transferListDoorbell, 1U << slot);

	if (logCommands) {
		std::println(std::cout, "block/ufs: Awaiting transfer on slot {}", slot);
	}

	co_await completion.event.wait();

	if (logCommands) {
		std::println(std::cout, "block/ufs: Transfer on slot {} done", slot);
	}
}

async::result<std::pair<transferStatus::Type, scsi::Error>> Controller::sendScsiCommand(const ScsiCommandInfo &info) {
	constexpr size_t responseUpiuSize = 64;

	assert(info.prdt.size() <= 0xffff);

	size_t size = sizeof(CommandUpiu) + responseUpiuSize + info.prdt.size_bytes();
	char *ptr = static_cast<char *>(dmaPool_.allocate(size, 1, 128));
	memset(ptr, 0, size);

	// fill in the command upiu
	assert(info.commandDataLength <= 16);
	arch::bit_value<uint8_t> flags = cmdUpiuFlags::taskAttr(CmdTaskAttr::simple);
	switch (info.dataDir) {
	case DataDirection::noData:
		break;
	case DataDirection::toTarget:
		flags |= cmdUpiuFlags::w(true);
		break;
	case DataDirection::fromTarget:
		flags |= cmdUpiuFlags::r(true);
		break;
	}

	auto *commandUpiu = new (ptr) CommandUpiu{
		.common{
			.transactionType = 1,
			.flags = flags,
			.lun = info.lun,
			.taskTag = 0,
			.commandSetType = SCSI_COMMAND_SET,
			.queryFunction = 0,
			.response = 0,
			.status = 0,
			.totalEhsLength = 0,
			.deviceInformation = 0,
			.dataSegmentLength = 0
		},
		.expectedDataTransferLength = info.dataLength,
		.cdb{}
	};
	memcpy(commandUpiu->cdb, info.commandData, info.commandDataLength);

	auto *prdtPtr = ptr + sizeof(CommandUpiu) + responseUpiuSize;
	memcpy(prdtPtr, info.prdt.data(), info.prdt.size_bytes());

	TransferInfo transferInfo{
		.ucdBase = helix::ptrToPhysical(ptr),
		.responseUpiuDwords = responseUpiuSize / 4,
		.responseUpiuOffsetDwords = sizeof(CommandUpiu) / 4,
		.physicalRegionDescCount = static_cast<uint16_t>(info.prdt.size()),
		.prdtOffsetDwords = (sizeof(CommandUpiu) + responseUpiuSize) / 4,
		.dataDir = info.dataDir
	};

	Completion completion;
	co_await sendTransfer_(transferInfo, completion);

	// todo scsi status
	auto response = reinterpret_cast<UpiuCommonHeader *>(ptr + sizeof(CommandUpiu));
	auto scsiError = scsi::statusToError(response->status);
	//std::println(std::cout, "scsi status: {:#x}", response->status);

	dmaPool_.deallocate(ptr, size, 1, 128);

	co_return std::make_pair(completion.status, scsiError);
}

void Controller::handleCompletion_(int slot) {
	if (logCommands) {
		std::println(std::cout, "block/ufs: Complete transfer on slot {}", slot);
	}

	auto completion = transferCompletions_[slot];
	auto &req = transferRequestList_->requests[slot];

	assert(completion);
	completion->status = req.status;
	completion->event.raise();

	transferCompletions_[slot] = nullptr;
	transferSlotsInUse_ &= ~(1U << slot);
}

async::detached Controller::run() {
	auto hce = regs_.load(regs::hce);
	hce |= hce::cge(true);
	hce |= hce::hce(true);
	regs_.store(regs::hce, hce);

	// reset the controller
	bool success = co_await helix::kindaBusyWait(1'000'000'000,
		[&] { return regs_.load(regs::hce) & hce::hce; });
	assert(success && "block/ufs: Controller reset timed out");

	auto cap = regs_.load(regs::cap);
	assert((cap & cap::as64) && "block/ufs: Controller doesn't support 64-bit addressing");

	numTaskSlots_ = (cap & cap::numTaskSlots) + 1;
	numTransferSlots_ = (cap & cap::numTransferSlots) + 1;

	// clear interrupt status bits
	regs_.store(regs::interruptStatus, regs_.load(regs::interruptStatus));

	// send DME_LINKSTARTUP UIC command
	regs_.store(regs::uicCmd, uicCmd::opcode(UicCommand::dmeLinkStartup));

	// wait for the command to complete
	success = co_await helix::kindaBusyWait(1'000'000'000,
		[&] { return regs_.load(regs::interruptStatus) & irqStatus::uccs; });
	assert(success && "block/ufs: DME link startup timed out");

	// clear interrupt status bits
	regs_.store(regs::interruptStatus, regs_.load(regs::interruptStatus));

	auto hcs = regs_.load(regs::hcs);
	assert((hcs & hcs::dp) && "block/ufs: No device present");

	// enable transfer completion interrupt
	auto irqEnable = regs_.load(regs::interruptEnable);
	irqEnable |= irqEnable::utrce(true);
	regs_.store(regs::interruptEnable, irqEnable);

	// disable interrupt aggregation
	auto utriacr = regs_.load(regs::utriacr);
	utriacr &= ~utriacr::iaen;
	regs_.store(regs::utriacr, utriacr);

	uintptr_t taskRequestListAddr = helix::ptrToPhysical(taskRequestList_.data());
	uintptr_t transferRequestListAddr = helix::ptrToPhysical(transferRequestList_.data());
	assert(taskRequestListAddr % 1024 == 0);
	assert(transferRequestListAddr % 1024 == 0);

	regs_.store(regs::taskRequestListLow, taskRequestListAddr);
	regs_.store(regs::taskRequestListHigh, taskRequestListAddr >> 32);

	regs_.store(regs::transferListBaseLow, transferRequestListAddr);
	regs_.store(regs::transferListBaseHigh, transferRequestListAddr >> 32);

	regs_.store(regs::taskRequestListRun, 1);
	regs_.store(regs::transferListRun, 1);

	handleIrqs_();

	// clear reset status
	auto senseData = co_await reportLunsUnit_.requestSense();
	(void)senseData;

	auto reportLunsResult = co_await reportLunsUnit_.reportLuns();
	assert(reportLunsResult && "block/ufs: Failed to get lun list");

	auto lunList = std::move(reportLunsResult.value());
	auto length = lunList[0];
	std::cout << "lun list reported len: " << length << std::endl;
	for (size_t i = 0; i < length / 8; ++i) {
		uint8_t lun = lunList[1 + i] >> 8;

		LogicalUnit tmp{this, lun};

		// clear reset status
		senseData = co_await tmp.requestSense();
		(void)senseData;

		auto capacityRes = co_await tmp.readCapacity();
		assert(capacityRes);
		auto capacityInfo = capacityRes.value();

		size_t capacity = (capacityInfo.lastAddressableLba + 1) * capacityInfo.logicalBlockSize;

		std::println(std::cout, "block/ufs: Found lun {} (size {} MiB, block size {})",
			static_cast<uint32_t>(lun), capacity / (1024 * 1024), capacityInfo.logicalBlockSize);

		logicalUnits_[i] = std::make_unique<LogicalStorageUnit>(this, parentId_, capacityInfo.logicalBlockSize, lun);
		logicalUnits_[i]->storageSize = capacity;
		logicalUnits_[i]->runScsi();

		blockfs::runDevice(logicalUnits_[i].get());
	}
}

async::detached Controller::handleIrqs_() {
	while (true) {
		if (logCommands) {
			std::println(std::cout, "block/ufs: Awaiting IRQ, seq {}, status {:x}",
				irqSequence_, static_cast<uint32_t>(regs_.load(regs::interruptStatus)));
		}

		auto await = co_await helix_ng::awaitEvent(irq_, irqSequence_);
		HEL_CHECK(await.error());
		irqSequence_ = await.sequence();

		if (logCommands) {
			std::println(std::cout, "block/ufs: Received IRQ, seq {}, status {:x}",
				irqSequence_, static_cast<uint32_t>(regs_.load(regs::interruptStatus)));
		}

		auto intStatus = regs_.load(regs::interruptStatus);
		if (static_cast<uint32_t>(intStatus)) {
			assert(intStatus & irqStatus::utrcs);

			regs_.store(regs::interruptStatus, intStatus);
			HEL_CHECK(helAcknowledgeIrq(irq_.getHandle(), kHelAckAcknowledge, irqSequence_));

			while (true) {
				auto inProgress = regs_.load(regs::transferListDoorbell);
				auto completedTransfers = transferSlotsInUse_ & ~inProgress;
				if (!completedTransfers) {
					break;
				}

				int numCompleted = 0;

				for (int i = 0; i < numTransferSlots_; ++i) {
					if (completedTransfers & 1U << i) {
						++numCompleted;

						handleCompletion_(i);

						regs_.store(regs::transferListCompletion, 1U << i);
					}
				}

				if (transfersInProgress_ >= numTransferSlots_
						&& transfersInProgress_ - numCompleted < numTransferSlots_) {
					freeTransferSlotDoorbell_.raise();
				}

				transfersInProgress_ -= numCompleted;
			}
		} else {
			HEL_CHECK(helAcknowledgeIrq(irq_.getHandle(), kHelAckNack, irqSequence_));
		}
	}
}
