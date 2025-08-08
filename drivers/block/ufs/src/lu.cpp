#include "lu.hpp"
#include "controller.hpp"

#include <print>
#include <unistd.h>

static size_t pageSize = getpagesize();

static async::result<frg::expected<scsi::Error, size_t>> sendScsiCommandImpl(Controller *ctrl, uint8_t lun, const scsi::CommandInfo &info) {
	if (info.isWrite) {
		/*std::println(std::cout, "block/ufs: Ignoring write request");
		co_return info.data.size();*/
	}

	assert(info.data.size() % 4 == 0);

	std::vector<PhysicalRegionDescriptor> prdt;
	for (size_t i = 0; i < info.data.size(); i += pageSize) {
		size_t remaining = info.data.size() - i;

		uint32_t count = remaining <= pageSize ? remaining : pageSize;

		uintptr_t phys = helix::ptrToPhysical(info.data.subview(i).data());
		assert(phys % 4 == 0);

		uint16_t dataByteCount = 0b11 | (((count / 4) - 1) << 2);
		PhysicalRegionDescriptor desc{
			.dataBaseLow = static_cast<uint32_t>(phys),
			.dataBaseHigh = static_cast<uint32_t>(phys >> 32),
			.reserved0 = 0,
			.dataByteCount = dataByteCount
		};
		prdt.push_back(desc);
	}

	ScsiCommandInfo scsiInfo{
		.lun = lun,
		.commandData = static_cast<const uint8_t *>(info.command.data()),
		.commandDataLength = static_cast<uint8_t>(info.command.size()),
		.dataDir = info.isWrite ? DataDirection::toTarget : DataDirection::fromTarget,
		.dataLength = static_cast<uint32_t>(info.data.size()),
		.prdt = prdt
	};
	auto result = co_await ctrl->sendScsiCommand(scsiInfo);
	assert(result.first == transferStatus::success);
	if (result.second.type != scsi::ErrorType::success)
		co_return result.second;

	co_return info.data.size();
}

async::result<frg::expected<scsi::Error, size_t>> LogicalUnit::sendScsiCommand(const scsi::CommandInfo &info) {
	co_return co_await sendScsiCommandImpl(ctrl_, lun_, info);
}

async::result<frg::expected<scsi::Error, size_t>> LogicalStorageUnit::sendScsiCommand(const scsi::CommandInfo &info) {
	co_return co_await sendScsiCommandImpl(ctrl_, lun_, info);
}
