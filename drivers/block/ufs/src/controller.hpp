#pragma once

#include <arch/mem_space.hpp>
#include <arch/dma_structs.hpp>
#include <arch/dma_pool.hpp>
#include <async/recurring-event.hpp>
#include <async/oneshot-event.hpp>
#include <async/result.hpp>
#include <helix/memory.hpp>
#include <protocols/hw/client.hpp>

#include <span>
#include <memory>

#include "spec.hpp"
#include "lu.hpp"

struct ScsiCommandInfo {
	uint8_t lun;
	const uint8_t *commandData;
	uint8_t commandDataLength;
	DataDirection dataDir;
	uint32_t dataLength;
	std::span<PhysicalRegionDescriptor> prdt;
};

class Controller {
public:
	Controller(int64_t parentId, protocols::hw::Device hwDevice, helix::Mapping ufsRegs, helix::UniqueDescriptor irq);

	async::detached run();

	async::result<std::pair<transferStatus::Type, scsi::Error>> sendScsiCommand(const ScsiCommandInfo &info);

private:
	struct TransferInfo {
		uintptr_t ucdBase;
		uint16_t responseUpiuDwords;
		uint16_t responseUpiuOffsetDwords;
		uint16_t physicalRegionDescCount;
		uint16_t prdtOffsetDwords;
		DataDirection dataDir;
	};

	struct Completion {
		async::oneshot_event event;
		transferStatus::Type status{};
	};

	async::result<int> findFreeTransferSlot_();
	async::result<void> sendTransfer_(const TransferInfo &info, Completion &completion);
	void handleCompletion_(int slot);

	async::detached handleIrqs_();

	protocols::hw::Device hwDevice_;
	helix::Mapping regsMapping_;
	arch::mem_space regs_;
	helix::UniqueDescriptor irq_;

	arch::contiguous_pool dmaPool_;
	arch::dma_object<TaskManagementRequestList> taskRequestList_;
	arch::dma_object<TransferRequestList> transferRequestList_;

	async::recurring_event freeTransferSlotDoorbell_;

	Completion *transferCompletions_[32];

	std::unique_ptr<LogicalStorageUnit> logicalUnits_[32];
	LogicalUnit reportLunsUnit_;

	int64_t parentId_;
	uint64_t irqSequence_;
	int numTaskSlots_;
	int numTransferSlots_;
	int transfersInProgress_;
	uint32_t transferSlotsInUse_;
};
