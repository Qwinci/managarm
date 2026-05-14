#include <string>
#include <locale>
#include <codecvt>

#include "protocols/usb/api.hpp"

namespace protocols::usb {

// ----------------------------------------------------------------------------
// Device.
// ----------------------------------------------------------------------------

Device::Device(std::shared_ptr<DeviceData> state)
: _state(std::move(state)) { }

arch::dma_pool *Device::setupPool() const {
	return _state->setupPool();
}

arch::dma_pool *Device::bufferPool() const {
	return _state->bufferPool();
}

async::result<frg::expected<UsbError, std::string>> Device::deviceDescriptor() const {
	return _state->deviceDescriptor();
}

async::result<frg::expected<UsbError, std::string>> Device::configurationDescriptor(uint8_t configuration) const {
	return _state->configurationDescriptor(configuration);
}

async::result<frg::expected<UsbError, uint8_t>> Device::currentConfigurationValue() const {
	arch::dma_object<SetupPacket> get{setupPool()};
	get->type = setup_type::targetDevice | setup_type::byStandard
			| setup_type::toHost;
	get->request = request_type::getConfig;
	get->value = 0;
	get->index = 0;
	get->length = 1;

	arch::dma_object<uint8_t> descriptor{bufferPool()};
	FRG_CO_TRY(co_await transfer(ControlTransfer{kXferToHost,
			get, descriptor.view_buffer()}));

	co_return *descriptor.data();
}

async::result<frg::expected<UsbError, Configuration>> Device::useConfiguration(uint8_t index, uint8_t value) const {
	return _state->useConfiguration(index, value);
}

async::result<frg::expected<UsbError, std::string>> Device::getString(size_t number) const {
	if(number == 0)
		co_return UsbError::unsupported;

	arch::dma_object<SetupPacket> desc{setupPool()};
	desc->type = setup_type::targetDevice | setup_type::byStandard | setup_type::toHost;
	desc->request = request_type::getDescriptor;
	desc->value = (descriptor_type::string << 8) | number;
	desc->index = 0x0409; // en-us
	desc->length = sizeof(StringDescriptor);

	arch::dma_object<StringDescriptor> header{bufferPool()};

	FRG_CO_TRY(co_await transfer(ControlTransfer(kXferToHost, desc, header.view_buffer())));

	desc->length = header->length;
	arch::dma_buffer buffer{bufferPool(), header->length};
	FRG_CO_TRY(co_await transfer(ControlTransfer(kXferToHost, desc, buffer)));

	std::wstring_convert<std::codecvt_utf8_utf16<char16_t>,char16_t> convert;
	auto res = reinterpret_cast<StringDescriptor *>(buffer.data());
	co_return convert.to_bytes(std::u16string{res->data, (res->length - sizeof(StringDescriptor)) / 2});
}

async::result<frg::expected<UsbError, size_t>> Device::transfer(ControlTransfer info) const {
	return _state->transfer(info);
}

// ----------------------------------------------------------------------------
// Configuration.
// ----------------------------------------------------------------------------

Configuration::Configuration(std::shared_ptr<ConfigurationData> state)
: _state(std::move(state)) { }

async::result<frg::expected<UsbError, Interface>> Configuration::useInterface(int number,
		int alternative) const {
	return _state->useInterface(number, alternative);
}

// ----------------------------------------------------------------------------
// Interface.
// ----------------------------------------------------------------------------

Interface::Interface(std::shared_ptr<InterfaceData> state)
: _state(std::move(state)) { }

async::result<frg::expected<UsbError, Endpoint>>
Interface::getEndpoint(PipeType type, int number) const {
	return _state->getEndpoint(type, number);
}

// ----------------------------------------------------------------------------
// Endpoint.
// ----------------------------------------------------------------------------

Endpoint::Endpoint(std::shared_ptr<EndpointData> state)
: _state(std::move(state)) { }

async::result<frg::expected<UsbError, size_t>> Endpoint::transfer(InterruptTransfer info) const {
	return _state->transfer(info);
}

async::result<frg::expected<UsbError, size_t>> Endpoint::transfer(BulkTransfer info) const {
	return _state->transfer(info);
}

// ----------------------------------------------------------------------------
// DeviceEndpoint
// ----------------------------------------------------------------------------

DeviceEndpoint::DeviceEndpoint(uint8_t number, PipeType type)
: number_{number}, type_{type} { }

async::result<frg::expected<UsbError>> DeviceEndpoint::enable(const EndpointDescriptor &descriptor,
		std::optional<SsEndpointCompanionDescriptor> ssDescriptor) {
	desc_ = descriptor;
	ssDesc_ = ssDescriptor;

	FRG_CO_TRY(co_await hwEnable_());
	enabled_ = true;
	co_return frg::success;
}

async::result<frg::expected<UsbError>> DeviceEndpoint::disable() {
	FRG_CO_TRY(co_await hwDisable_());
	enabled_ = false;
	co_return frg::success;
}

// ----------------------------------------------------------------------------
// DeviceController
// ----------------------------------------------------------------------------

async::result<frg::expected<UsbError>> DeviceController::start(DeviceGadget *gadget) {
	boundGadget_ = gadget;
	gadget->boundController_ = this;
	co_return co_await hwStart_();
}

async::result<frg::expected<UsbError>> DeviceController::stop() {
	FRG_CO_TRY(co_await hwStop_());
	boundGadget_->boundController_ = nullptr;
	boundGadget_ = nullptr;
	co_return frg::success;
}

} // namespace protocols::usb
