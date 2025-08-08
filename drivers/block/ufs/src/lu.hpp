#pragma once

#include <scsi.hpp>

class Controller;

class LogicalUnit : public scsi::Interface {
public:
	LogicalUnit(Controller *ctrl, uint8_t lun) : ctrl_{ctrl}, lun_{lun} { }

	async::result<frg::expected<scsi::Error, size_t>> sendScsiCommand(const scsi::CommandInfo &info) override;

private:
	Controller *ctrl_;
	uint8_t lun_;
};

class LogicalStorageUnit : public scsi::StorageDevice {
public:
	LogicalStorageUnit(Controller *ctrl, int64_t parentId, size_t sectorSize, uint8_t lun)
		: scsi::StorageDevice{sectorSize, parentId}, ctrl_{ctrl}, lun_{lun} { }

	async::result<frg::expected<scsi::Error, size_t>> sendScsiCommand(const scsi::CommandInfo &info) override;

private:
	Controller *ctrl_;
	uint8_t lun_;
};
