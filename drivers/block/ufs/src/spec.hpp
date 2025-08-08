#pragma once

#include <arch/bits.hpp>
#include <arch/variable.hpp>

#include <cstdint>

inline constexpr uint32_t TASK_DW0_IRQ_BIT = 1 << 0;
inline constexpr uint32_t TASK_STATUS_MASK = 0xff;

namespace taskStatus {

enum : uint8_t {
	success = 0,
	invalidFunctionAttribs = 1,
	requestSizeMismatch = 2,
	responseSizeMismatch = 3,
	peerCommunicationFailure = 4,
	aborted = 5,
	fatalError = 6,
	fatalDeviceError = 7,
	invalidOcsValue = 0xf
};

} // namespace taskStatus

struct TaskManagementRequest {
	uint32_t dw0;
	uint32_t reserved0;
	uint32_t status;
	uint32_t reserved1;
	uint32_t requestUpiu[8];
	uint32_t responseUpiu[8];
};

struct alignas(1024) TaskManagementRequestList {
	TaskManagementRequest requests[8];
};

struct TransferRequest {
	arch::bit_value<uint32_t> flags{0};
	uint32_t dataUnitNumLower;
	uint32_t status;
	uint32_t dataUnitNumUpper;
	uint32_t ucdBaseLower;
	uint32_t ucdBaseUpper;
	uint16_t responseUpiuLength;
	uint16_t responseUpiuOffset;
	uint16_t prdtLength;
	uint16_t prdtOffset;
};

struct alignas(1024) TransferRequestList {
	TransferRequest requests[32];
};

inline constexpr uint8_t CMD_TYPE_UFS_STORAGE = 1;

enum class DataDirection : uint8_t {
	noData = 0,
	toTarget = 1,
	fromTarget = 2
};

namespace transferFlags {

inline constexpr arch::field<uint32_t, uint8_t> cryptoConfigIndex{0, 8};
inline constexpr arch::field<uint32_t, bool> cryptoEnable{23, 1};
inline constexpr arch::field<uint32_t, bool> interrupt{24, 1};
inline constexpr arch::field<uint32_t, DataDirection> dir{25, 2};
inline constexpr arch::field<uint32_t, uint8_t> commandType{28, 4};

} // namespace transferFlags

inline constexpr uint32_t TRANSFER_STATUS_MASK = 0xff;

namespace transferStatus {

using Type = uint8_t;

enum : uint8_t {
	success = 0,
	invalidCommandTableAttribs = 1,
	invalidPrdtAttribs = 2,
	dataBufferSizeMismatch = 3,
	responseUpiuSizeMismatch = 4,
	communicationFailure = 5,
	aborted = 6,
	fatalError = 7,
	fatalDeviceError = 8,
	invalidCryptoConfig = 9,
	generalCryptoError = 0xa,
	invalidOcsValue = 0xf
};

} // namespace transferStatus

struct PhysicalRegionDescriptor {
	uint32_t dataBaseLow;
	uint32_t dataBaseHigh;
	uint32_t reserved0;
	uint32_t dataByteCount;
};

inline constexpr uint16_t PRD_DATA_BYTE_COUNT_DWORD_GRAN = 0b11;

struct UpiuCommonHeader {
	uint8_t transactionType;
	arch::bit_value<uint8_t> flags;
	uint8_t lun;
	uint8_t taskTag;
	uint8_t commandSetType;
	union {
		uint8_t queryFunction;
		uint8_t taskManagementFunction;
	};
	uint8_t response;
	uint8_t status;
	uint8_t totalEhsLength;
	uint8_t deviceInformation;
	arch::scalar_storage<uint16_t, arch::big_endian> dataSegmentLength;
};

struct CommandUpiu {
	UpiuCommonHeader common;
	arch::scalar_storage<uint32_t, arch::big_endian> expectedDataTransferLength;
	uint32_t cdb[4];
};

inline constexpr uint8_t SCSI_COMMAND_SET = 0;

enum class CmdTaskAttr : uint8_t {
	simple = 0,
	ordered = 1,
	headOfQueue = 2
};

namespace cmdUpiuFlags {

inline constexpr arch::field<uint8_t, CmdTaskAttr> taskAttr{0, 2};
inline constexpr arch::field<uint8_t, bool> cp{2, 1};
inline constexpr arch::field<uint8_t, bool> w{5, 1};
inline constexpr arch::field<uint8_t, bool> r{6, 1};

} // namespace cmdUpiuFlags

namespace responseUpiuFlags {

inline constexpr arch::field<uint8_t, bool> d{4, 1};
inline constexpr arch::field<uint8_t, bool> u{5, 1};
inline constexpr arch::field<uint8_t, bool> o{6, 1};

} // namespace responseUpiuFlags
