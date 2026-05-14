#pragma once

#include <memory>
#include <vector>
#include <optional>

#include <arch/dma_structs.hpp>
#include <async/result.hpp>
#include <async/generator.hpp>
#include <frg/expected.hpp>

#include "usb.hpp"

namespace protocols::usb {

enum class UsbError {
	none,
	stall,
	babble,
	timeout,
	unsupported,
	other
};

enum class DeviceSpeed {
	lowSpeed,
	fullSpeed,
	highSpeed,
	superSpeed
};

enum XferFlags {
	kXferToDevice = 1,
	kXferToHost = 2
};

struct ControlTransfer {
	ControlTransfer(XferFlags flags, arch::dma_object_view<SetupPacket> setup,
			arch::dma_buffer_view buffer)
	: flags{flags}, setup{setup}, buffer{buffer} { }

	XferFlags flags;
	arch::dma_object_view<SetupPacket> setup;
	arch::dma_buffer_view buffer;
};

struct InterruptTransfer {
	InterruptTransfer(XferFlags flags, arch::dma_buffer_view buffer)
	: flags{flags}, buffer{buffer},
			allowShortPackets{false}, lazyNotification{false} { }

	XferFlags flags;
	arch::dma_buffer_view buffer;
	bool allowShortPackets;
	bool lazyNotification;
};

struct BulkTransfer {
	BulkTransfer(XferFlags flags, arch::dma_buffer_view buffer)
	: flags{flags}, buffer{buffer},
			allowShortPackets{false}, lazyNotification{false} { }

	XferFlags flags;
	arch::dma_buffer_view buffer;
	bool allowShortPackets;
	bool lazyNotification;
};

enum class PipeType {
	null, in, out, control
};

// ----------------------------------------------------------------------------
// EndpointData
// ----------------------------------------------------------------------------

struct EndpointData {
protected:
	~EndpointData() = default;

public:
	virtual async::result<frg::expected<UsbError, size_t>> transfer(ControlTransfer info) = 0;
	virtual async::result<frg::expected<UsbError, size_t>> transfer(InterruptTransfer info) = 0;
	virtual async::result<frg::expected<UsbError, size_t>> transfer(BulkTransfer info) = 0;
};


struct Endpoint {
	Endpoint(std::shared_ptr<EndpointData> state);

	async::result<frg::expected<UsbError, size_t>> transfer(ControlTransfer info) const;
	async::result<frg::expected<UsbError, size_t>> transfer(InterruptTransfer info) const;
	async::result<frg::expected<UsbError, size_t>> transfer(BulkTransfer info) const;

private:
	std::shared_ptr<EndpointData> _state;
};

// ----------------------------------------------------------------------------
// InterfaceData
// ----------------------------------------------------------------------------

struct InterfaceData {
protected:
	InterfaceData(int num) : interface_{num} { }

	~InterfaceData() = default;

public:
	virtual async::result<frg::expected<UsbError, Endpoint>>
	getEndpoint(PipeType type, int number) = 0;

	int interface() const {
		return interface_;
	}
private:
	int interface_;
};

struct Interface {
	Interface(std::shared_ptr<InterfaceData> state);

	async::result<frg::expected<UsbError, Endpoint>>
	getEndpoint(PipeType type, int number) const;

	int num() {
		return _state->interface();
	}
private:
	std::shared_ptr<InterfaceData> _state;
};


// ----------------------------------------------------------------------------
// ConfigurationData
// ----------------------------------------------------------------------------

struct ConfigurationData {
protected:
	~ConfigurationData() = default;

public:
	virtual async::result<frg::expected<UsbError, Interface>>
	useInterface(int number, int alternative) = 0;
};

struct Configuration {
	Configuration(std::shared_ptr<ConfigurationData> state);

	async::result<frg::expected<UsbError, Interface>>
	useInterface(int number, int alternative) const;

private:
	std::shared_ptr<ConfigurationData> _state;
};

// ----------------------------------------------------------------------------
// DeviceData
// ----------------------------------------------------------------------------

struct DeviceData {
protected:
	~DeviceData() = default;

public:
	virtual arch::dma_pool *setupPool() = 0;
	virtual arch::dma_pool *bufferPool() = 0;

	virtual async::result<frg::expected<UsbError, std::string>> deviceDescriptor() = 0;
	virtual async::result<frg::expected<UsbError, std::string>> configurationDescriptor(uint8_t configuration) = 0;
	virtual async::result<frg::expected<UsbError, Configuration>> useConfiguration(uint8_t index, uint8_t value) = 0;
	virtual async::result<frg::expected<UsbError, size_t>> transfer(ControlTransfer info) = 0;
};

struct Device {
	Device(std::shared_ptr<DeviceData> state);

	arch::dma_pool *setupPool() const;
	arch::dma_pool *bufferPool() const;

	async::result<frg::expected<UsbError, std::string>> deviceDescriptor() const;
	async::result<frg::expected<UsbError, std::string>> configurationDescriptor(uint8_t configuration) const;
	async::result<frg::expected<UsbError, uint8_t>> currentConfigurationValue() const;
	async::result<frg::expected<UsbError, Configuration>> useConfiguration(uint8_t index, uint8_t value) const;
	async::result<frg::expected<UsbError, std::string>> getString(size_t number) const;
	async::result<frg::expected<UsbError, size_t>> transfer(ControlTransfer info) const;

	std::shared_ptr<DeviceData> state() const {
		return _state;
	}

private:
	std::shared_ptr<DeviceData> _state;
};

// ----------------------------------------------------------------
// BaseController.
// ----------------------------------------------------------------

struct Hub;

struct BaseController {
protected:
	~BaseController() = default;

public:
	virtual async::result<frg::expected<UsbError>>
	enumerateDevice(std::shared_ptr<Hub> hub, int port, DeviceSpeed speed) = 0;
};

inline std::string getSpeedMbps(DeviceSpeed speed) {
	switch(speed) {
		case DeviceSpeed::fullSpeed: {
			return "12";
		}
		case DeviceSpeed::lowSpeed: {
			return "1.5";
		}
		case DeviceSpeed::highSpeed: {
			return "480";
		}
		case DeviceSpeed::superSpeed: {
			return "5000";
		}
		default: {
			return "unknown";
		}
	}
}

struct DeviceController;

// ----------------------------------------------------------------------------
// DeviceGadget
// ----------------------------------------------------------------------------

struct DeviceGadget {
	virtual ~DeviceGadget() = default;

	virtual async::generator<frg::expected<UsbError, std::vector<std::byte> *>>
	processSetupPacket(SetupPacket packet) = 0;

	DeviceController *getController() {
		return boundController_;
	}

private:
	friend DeviceController;

	DeviceController *boundController_{};
};

// ----------------------------------------------------------------------------
// DeviceEndpoint
// ----------------------------------------------------------------------------

struct DeviceEndpoint {
	DeviceEndpoint(uint8_t number, PipeType type);
	virtual ~DeviceEndpoint() = default;

	async::result<frg::expected<UsbError>> enable(const EndpointDescriptor &descriptor,
			std::optional<SsEndpointCompanionDescriptor> ssDescriptor);
	async::result<frg::expected<UsbError>> disable();

	virtual async::result<frg::expected<UsbError>> setStall(bool stall) = 0;

	virtual async::result<frg::expected<UsbError, size_t>> transfer(ControlTransfer info) = 0;
	virtual async::result<frg::expected<UsbError, size_t>> transfer(InterruptTransfer info) = 0;
	virtual async::result<frg::expected<UsbError, size_t>> transfer(BulkTransfer info) = 0;

	const EndpointDescriptor &descriptor() const {
		return desc_;
	}

	const std::optional<SsEndpointCompanionDescriptor> &ssDescriptor() const {
		return ssDesc_;
	}

	uint8_t number() const {
		return number_;
	}

	PipeType type() const {
		return type_;
	}

	bool enabled() const {
		return enabled_;
	}

protected:
	virtual async::result<frg::expected<UsbError>> hwEnable_() = 0;
	virtual async::result<frg::expected<UsbError>> hwDisable_() = 0;

private:
	EndpointDescriptor desc_{};
	std::optional<SsEndpointCompanionDescriptor> ssDesc_{};
	uint8_t number_{};
	PipeType type_{};
	bool enabled_{};
};

// ----------------------------------------------------------------------------
// DeviceController
// ----------------------------------------------------------------------------

struct DeviceController {
	virtual ~DeviceController() = default;

	async::result<frg::expected<UsbError>> start(DeviceGadget *gadget);
	async::result<frg::expected<UsbError>> stop();

	size_t supportedEndpoints() const {
		return eps_.size();
	}

	// EP0/EP1 CONTROL, EP2=IN, EP3=OUT, etc.
	// OUT = host to device, IN = device to host
	DeviceEndpoint *getEndpoint(uint8_t number) const {
		if (number >= eps_.size())
			return nullptr;
		return eps_[number].get();
	}

	DeviceGadget *getGadget() const {
		return boundGadget_;
	}

	DeviceSpeed getSpeed() const {
		return speed_;
	}

protected:
	virtual async::result<frg::expected<UsbError>> hwStart_() = 0;
	virtual async::result<frg::expected<UsbError>> hwStop_() = 0;

	std::vector<std::unique_ptr<DeviceEndpoint>> eps_;
	DeviceGadget *boundGadget_{};
	DeviceSpeed speed_{DeviceSpeed::superSpeed};
};

} // namespace protocols::usb
