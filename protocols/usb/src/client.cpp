
#include <memory>
#include <iostream>

#include <string.h>

#include <async/result.hpp>
#include <helix/ipc.hpp>

#include <bragi/helpers-std.hpp>
#include "usb.bragi.hpp"
#include "protocols/usb/client.hpp"

namespace protocols::usb {

namespace {

struct DeviceState final : DeviceData {
	DeviceState(helix::UniqueLane lane)
	:_lane(std::move(lane)) { }

	arch::dma_pool *setupPool() override;
	arch::dma_pool *bufferPool() override;

	async::result<frg::expected<UsbError, std::string>> deviceDescriptor() override;
	async::result<frg::expected<UsbError, std::string>> configurationDescriptor(uint8_t configuration) override;
	async::result<frg::expected<UsbError, Configuration>> useConfiguration(uint8_t index, uint8_t value) override;
	async::result<frg::expected<UsbError, size_t>> transfer(ControlTransfer info) override;

private:
	helix::UniqueLane _lane;
};

struct ConfigurationState final : ConfigurationData {
	ConfigurationState(helix::UniqueLane lane)
	:_lane(std::move(lane)) { }

	async::result<frg::expected<UsbError, Interface>>
	useInterface(int number, int alternative) override;

private:
	helix::UniqueLane _lane;
};

struct InterfaceState final : InterfaceData {
	InterfaceState(int num, helix::UniqueLane lane)
	: InterfaceData{num}, _lane(std::move(lane)) { }

	async::result<frg::expected<UsbError, Endpoint>>
	getEndpoint(PipeType type, int number) override;

private:
	helix::UniqueLane _lane;
};


struct EndpointState final : EndpointData {
	EndpointState(helix::UniqueLane lane)
	:_lane(std::move(lane)) { }

	async::result<frg::expected<UsbError, size_t>> transfer(ControlTransfer info) override;
	async::result<frg::expected<UsbError, size_t>> transfer(InterruptTransfer info) override;
	async::result<frg::expected<UsbError, size_t>> transfer(BulkTransfer info) override;

private:
	helix::UniqueLane _lane;
};

struct RemoteDeviceGadgetServer final : DeviceGadget {
	RemoteDeviceGadgetServer(helix::UniqueLane lane) : lane_{std::move(lane)} { }

	async::generator<frg::expected<UsbError, std::vector<std::byte> *>> processSetupPacket(SetupPacket packet) override;

private:
	helix::UniqueLane lane_;
};

arch::dma_pool *DeviceState::setupPool() {
	return nullptr;
}

arch::dma_pool *DeviceState::bufferPool() {
	return nullptr;
}

frg::expected<UsbError> transformProtocolError(managarm::usb::Errors error) {
	switch (error) {
		using enum UsbError;
		using enum managarm::usb::Errors;

		case SUCCESS: return frg::success;
		case STALL: return stall;
		case BABBLE: return babble;
		case TIMEOUT: return timeout;
		case UNSUPPORTED: return unsupported;
		case OTHER: return other;
		case ILLEGAL_REQUEST: assert(!"Illegal request in USB client"); break;
		default: assert(!"Invalid error code in protocolErrorIntoApiError");
	}

	return UsbError::other;
}

managarm::usb::Errors toProtocolError(UsbError error) {
	switch (error) {
		using enum UsbError;
		using enum managarm::usb::Errors;

		case none: return SUCCESS;
		case stall: return STALL;
		case babble: return BABBLE;
		case timeout: return TIMEOUT;
		case unsupported: return UNSUPPORTED;
		case other: return OTHER;
		default: assert(!"Invalid error code in toProtocolError");
	}
}

async::result<frg::expected<UsbError, std::string>> DeviceState::deviceDescriptor() {
	managarm::usb::GetDeviceDescriptorRequest req;

	auto [offer, sendReq, recvResp, recvData] = co_await helix_ng::exchangeMsgs(
		_lane,
		helix_ng::offer(
			helix_ng::sendBragiHeadOnly(req, frg::stl_allocator{}),
			helix_ng::recvInline(),
			helix_ng::recvInline()
		)
	);

	HEL_CHECK(offer.error());
	HEL_CHECK(sendReq.error());
	HEL_CHECK(recvResp.error());

	auto resp = bragi::parse_head_only<managarm::usb::SvrResponse>(recvResp);
	recvResp.reset();

	FRG_CO_TRY(transformProtocolError(resp->error()));

	HEL_CHECK(recvData.error());

	std::string data(recvData.length(), 0);
	memcpy(&data[0], recvData.data(), recvData.length());
	recvData.reset();
	co_return std::move(data);
}

async::result<frg::expected<UsbError, std::string>> DeviceState::configurationDescriptor(uint8_t configuration) {
	managarm::usb::GetConfigurationDescriptorRequest req;
	req.set_configuration(configuration);

	auto [offer, sendReq, recvResp] = co_await helix_ng::exchangeMsgs(
		_lane,
		helix_ng::offer(
			helix_ng::want_lane,
			helix_ng::sendBragiHeadOnly(req, frg::stl_allocator{}),
			helix_ng::recvInline()
		)
	);

	HEL_CHECK(offer.error());
	HEL_CHECK(sendReq.error());
	HEL_CHECK(recvResp.error());

	auto resp = bragi::parse_head_only<managarm::usb::SvrResponse>(recvResp);
	recvResp.reset();

	std::string recvBuffer(resp->size(), 0);
	auto [recvData] = co_await helix_ng::exchangeMsgs(
		offer.descriptor(),
		helix_ng::recvBuffer(recvBuffer.data(), recvBuffer.size())
	);

	FRG_CO_TRY(transformProtocolError(resp->error()));

	HEL_CHECK(recvData.error());

	co_return recvBuffer;
}

async::result<frg::expected<UsbError, Configuration>> DeviceState::useConfiguration(uint8_t index, uint8_t value) {
	managarm::usb::UseConfigurationRequest req;
	req.set_index(index);
	req.set_value(value);

	auto [offer, sendReq, recvResp, pullLane] = co_await helix_ng::exchangeMsgs(
		_lane,
		helix_ng::offer(
			helix_ng::sendBragiHeadOnly(req, frg::stl_allocator{}),
			helix_ng::recvInline(),
			helix_ng::pullDescriptor()
		)
	);

	HEL_CHECK(offer.error());
	HEL_CHECK(sendReq.error());
	HEL_CHECK(recvResp.error());

	auto resp = bragi::parse_head_only<managarm::usb::SvrResponse>(recvResp);
	recvResp.reset();

	FRG_CO_TRY(transformProtocolError(resp->error()));

	HEL_CHECK(pullLane.error());

	auto state = std::make_shared<ConfigurationState>(pullLane.descriptor());
	co_return Configuration(std::move(state));
}

async::result<frg::expected<UsbError, size_t>>
doControlTransfer(auto &lane, ControlTransfer info) {
	managarm::usb::TransferRequest req;

	req.set_type(managarm::usb::XferType::CONTROL);
	req.set_dir(info.flags == kXferToDevice
		    ? managarm::usb::XferDirection::TO_DEVICE
		    : managarm::usb::XferDirection::TO_HOST);

	req.set_length(info.buffer.size());

	if(info.flags == kXferToDevice) {
		auto [offer, sendReq, sendSetup, sendData, recvResp] = co_await helix_ng::exchangeMsgs(
			lane,
			helix_ng::offer(
				helix_ng::sendBragiHeadOnly(req, frg::stl_allocator{}),
				helix_ng::sendBuffer(info.setup.data(), sizeof(SetupPacket)),
				helix_ng::sendBuffer(info.buffer.data(), info.buffer.size()),
				helix_ng::recvInline()
			)
		);

		HEL_CHECK(offer.error());
		HEL_CHECK(sendReq.error());
		HEL_CHECK(sendSetup.error());
		HEL_CHECK(sendData.error());
		HEL_CHECK(recvResp.error());

		auto resp = bragi::parse_head_only<managarm::usb::SvrResponse>(recvResp);
		recvResp.reset();

		FRG_CO_TRY(transformProtocolError(resp->error()));

		co_return 0;
	}else{
		auto [offer, sendReq, sendSetup, recvResp, recvData] = co_await helix_ng::exchangeMsgs(
			lane,
			helix_ng::offer(
				helix_ng::sendBragiHeadOnly(req, frg::stl_allocator{}),
				helix_ng::sendBuffer(info.setup.data(), sizeof(SetupPacket)),
				helix_ng::recvInline(),
				helix_ng::recvBuffer(info.buffer.data(), info.buffer.size())
			)
		);

		HEL_CHECK(offer.error());
		HEL_CHECK(sendReq.error());
		HEL_CHECK(sendSetup.error());
		HEL_CHECK(recvResp.error());

		auto resp = bragi::parse_head_only<managarm::usb::SvrResponse>(recvResp);
		recvResp.reset();

		FRG_CO_TRY(transformProtocolError(resp->error()));

		HEL_CHECK(recvData.error());

		co_return resp->size();
	}
}

async::result<frg::expected<UsbError, size_t>> DeviceState::transfer(ControlTransfer info) {
	co_return co_await doControlTransfer(_lane, info);
}

async::result<frg::expected<UsbError, Interface>>
ConfigurationState::useInterface(int number, int alternative) {
	managarm::usb::UseInterfaceRequest req;

	req.set_number(number);
	req.set_alternative(alternative);

	auto [offer, sendReq, recvResp, pullLane] = co_await helix_ng::exchangeMsgs(
		_lane,
		helix_ng::offer(
			helix_ng::sendBragiHeadOnly(req, frg::stl_allocator{}),
			helix_ng::recvInline(),
			helix_ng::pullDescriptor()
		)
	);

	HEL_CHECK(offer.error());
	HEL_CHECK(sendReq.error());
	HEL_CHECK(recvResp.error());

	auto resp = bragi::parse_head_only<managarm::usb::SvrResponse>(recvResp);
	recvResp.reset();

	FRG_CO_TRY(transformProtocolError(resp->error()));

	HEL_CHECK(pullLane.error());

	auto state = std::make_shared<InterfaceState>(number, pullLane.descriptor());
	co_return Interface(std::move(state));
}

async::result<frg::expected<UsbError, Endpoint>>
InterfaceState::getEndpoint(PipeType type, int number) {
	managarm::usb::GetEndpointRequest req;

	req.set_type(static_cast<int>(type));
	req.set_number(number);

	auto [offer, sendReq, recvResp, pullLane] = co_await helix_ng::exchangeMsgs(
		_lane,
		helix_ng::offer(
			helix_ng::sendBragiHeadOnly(req, frg::stl_allocator{}),
			helix_ng::recvInline(),
			helix_ng::pullDescriptor()
		)
	);

	HEL_CHECK(offer.error());
	HEL_CHECK(sendReq.error());
	HEL_CHECK(recvResp.error());

	auto resp = bragi::parse_head_only<managarm::usb::SvrResponse>(recvResp);
	recvResp.reset();

	FRG_CO_TRY(transformProtocolError(resp->error()));

	HEL_CHECK(pullLane.error());

	auto state = std::make_shared<EndpointState>(pullLane.descriptor());
	co_return Endpoint(std::move(state));
}

template <typename XferInfo>
async::result<frg::expected<UsbError, size_t>>
doTransferOfType(auto &lane, managarm::usb::XferType type, XferInfo info) {
	managarm::usb::TransferRequest req;

	req.set_type(type);
	req.set_dir(info.flags == kXferToDevice
		    ? managarm::usb::XferDirection::TO_DEVICE
		    : managarm::usb::XferDirection::TO_HOST);

	req.set_allow_short_packets(info.allowShortPackets);
	req.set_lazy_notification(info.lazyNotification);
	req.set_length(info.buffer.size());

	if(info.flags == kXferToDevice) {
		auto [offer, sendReq, sendData, recvResp] =
			co_await helix_ng::exchangeMsgs(
				lane,
				helix_ng::offer(
					helix_ng::sendBragiHeadOnly(req, frg::stl_allocator{}),
					helix_ng::sendBuffer(info.buffer.data(), info.buffer.size()),
					helix_ng::recvInline()
				)
			);

		HEL_CHECK(offer.error());
		HEL_CHECK(sendReq.error());
		HEL_CHECK(sendData.error());
		HEL_CHECK(recvResp.error());

		auto resp = bragi::parse_head_only<managarm::usb::SvrResponse>(recvResp);
		recvResp.reset();

		FRG_CO_TRY(transformProtocolError(resp->error()));

		co_return resp->size();
	}else{
		auto [offer, sendReq, recvResp, recvData] = co_await helix_ng::exchangeMsgs(
			lane,
			helix_ng::offer(
				helix_ng::sendBragiHeadOnly(req, frg::stl_allocator{}),
				helix_ng::recvInline(),
				helix_ng::recvBuffer(info.buffer.data(), info.buffer.size())
			)
		);

		HEL_CHECK(offer.error());
		HEL_CHECK(sendReq.error());
		HEL_CHECK(recvResp.error());

		auto resp = bragi::parse_head_only<managarm::usb::SvrResponse>(recvResp);
		recvResp.reset();

		FRG_CO_TRY(transformProtocolError(resp->error()));

		HEL_CHECK(recvData.error());

		co_return recvData.actualLength();
	}
}

async::result<frg::expected<UsbError, size_t>> EndpointState::transfer(ControlTransfer info) {
	co_return co_await doControlTransfer(_lane, info);
}

async::result<frg::expected<UsbError, size_t>> EndpointState::transfer(InterruptTransfer info) {
	co_return co_await doTransferOfType(_lane, managarm::usb::XferType::INTERRUPT, info);
}

async::result<frg::expected<UsbError, size_t>> EndpointState::transfer(BulkTransfer info) {
	co_return co_await doTransferOfType(_lane, managarm::usb::XferType::BULK, info);
}

async::generator<frg::expected<UsbError, std::vector<std::byte> *>>
RemoteDeviceGadgetServer::processSetupPacket(SetupPacket packet) {
	managarm::usb::VerifySetupPacketRequest verifySetupReq;

	verifySetupReq.set_type(packet.type);
	verifySetupReq.set_request(packet.request);
	verifySetupReq.set_value(packet.value);
	verifySetupReq.set_index(packet.index);
	verifySetupReq.set_length(packet.length);

	auto [verifySetupOffer, verifySetupSendReq, verifySetupRecvResp] = co_await helix_ng::exchangeMsgs(
		lane_,
		helix_ng::offer(
			helix_ng::want_lane,
			helix_ng::sendBragiHeadOnly(verifySetupReq, frg::stl_allocator{}),
			helix_ng::recvInline()
		)
	);
	HEL_CHECK(verifySetupOffer.error());
	HEL_CHECK(verifySetupSendReq.error());
	HEL_CHECK(verifySetupRecvResp.error());

	auto conversation = verifySetupOffer.descriptor();

	auto preamble = bragi::read_preamble(verifySetupRecvResp);
	if (preamble.error()) {
		std::cout << "RemoteDeviceGadgetServer::processSetupPacket: error decoding preamble" << std::endl;
		auto [dismiss] = co_await helix_ng::exchangeMsgs(
			conversation, helix_ng::dismiss());
		HEL_CHECK(dismiss.error());
		co_yield UsbError::other;
		co_return;
	}

	std::vector<uint8_t> tail(preamble.tail_size());
	auto [recv_tail] = co_await helix_ng::exchangeMsgs(
		conversation,
		helix_ng::recvBuffer(tail.data(), tail.size())
	);
	HEL_CHECK(recv_tail.error());

	auto resp = *bragi::parse_head_tail<managarm::usb::VerifySetupPacketResponse>(verifySetupRecvResp, tail);
	verifySetupRecvResp.reset();

	auto status = transformProtocolError(resp.error());
	if (!status) {
		co_yield status.error();
		co_return;
	}

	if (!resp.has_data()) {
		co_yield nullptr;
		co_return;
	}

	std::vector<std::byte> data(resp.data_size());
	memcpy(data.data(), resp.data().data(), resp.data_size());
	co_yield &data;

	managarm::usb::VerifySetupPacketDataRequest verifyDataReq;
	verifyDataReq.data().resize(data.size());
	memcpy(verifyDataReq.data().data(), data.data(), data.size());

	auto [verifyDataSendReq, verifyDataSendTailReq, verifyDataRecvResp] = co_await helix_ng::exchangeMsgs(
		conversation,
		helix_ng::sendBragiHeadTail(verifyDataReq, frg::stl_allocator{}),
		helix_ng::recvInline()
	);
	HEL_CHECK(verifyDataSendReq.error());
	HEL_CHECK(verifyDataSendTailReq.error());
	HEL_CHECK(verifyDataRecvResp.error());

	auto verifyDataResp = *bragi::parse_head_only<managarm::usb::VerifySetupPacketDataResponse>(verifyDataRecvResp);
	verifyDataRecvResp.reset();

	status = transformProtocolError(verifyDataResp.error());
	if (!status) {
		co_yield status.error();
		co_return;
	}

	co_yield nullptr;
	co_return;
}

struct RemoteDeviceController final : DeviceController {
	RemoteDeviceController(helix::UniqueLane lane) : lane_{std::move(lane)} { }

	async::result<frg::expected<UsbError>> hwStart_() override;
	async::result<frg::expected<UsbError>> hwStop_() override;

private:
	async::detached run();

	async::result<void> processSetupPacket(helix_ng::BorrowedDescriptor conversation, managarm::usb::VerifySetupPacketRequest &verifySetupReq);

	helix::UniqueLane lane_;
};

async::result<frg::expected<UsbError>> RemoteDeviceController::hwStart_() {
	managarm::usb::StartDeviceControllerRequest req;

	auto [offer, sendReq, recvResp] = co_await helix_ng::exchangeMsgs(
		lane_,
		helix_ng::offer(
			helix_ng::sendBragiHeadOnly(req, frg::stl_allocator{}),
			helix_ng::recvInline()
		)
	);
	HEL_CHECK(offer.error());
	HEL_CHECK(sendReq.error());
	HEL_CHECK(recvResp.error());

	auto resp = *bragi::parse_head_only<managarm::usb::StartDeviceControllerResponse>(recvResp);

	if (resp.error() == managarm::usb::Errors::SUCCESS)
		run();

	co_return transformProtocolError(resp.error());
}

async::result<frg::expected<UsbError>> RemoteDeviceController::hwStop_() {
	managarm::usb::StopDeviceControllerRequest req;

	auto [offer, sendReq, recvResp] = co_await helix_ng::exchangeMsgs(
		lane_,
		helix_ng::offer(
			helix_ng::sendBragiHeadOnly(req, frg::stl_allocator{}),
			helix_ng::recvInline()
		)
	);
	HEL_CHECK(offer.error());
	HEL_CHECK(sendReq.error());
	HEL_CHECK(recvResp.error());

	auto resp = *bragi::parse_head_only<managarm::usb::StopDeviceControllerResponse>(recvResp);
	co_return transformProtocolError(resp.error());
}

async::result<void> RemoteDeviceController::processSetupPacket(helix_ng::BorrowedDescriptor conversation,
		managarm::usb::VerifySetupPacketRequest &verifySetupReq) {
	SetupPacket packet{
		.type = verifySetupReq.type(),
		.request = verifySetupReq.request(),
		.value = verifySetupReq.value(),
		.index = verifySetupReq.index(),
		.length = verifySetupReq.length()
	};
	auto gen = getGadget()->processSetupPacket(packet);

	auto result = co_await gen.next();

	managarm::usb::VerifySetupPacketResponse verifySetupResp;

	if (!result) {
		verifySetupResp.set_error(toProtocolError(result->error()));

		auto [verifySetupSendResp, verifySetupSendRespTail] = co_await helix_ng::exchangeMsgs(
			conversation,
			helix_ng::sendBragiHeadTail(verifySetupResp, frg::stl_allocator{})
		);
		HEL_CHECK(verifySetupSendResp.error());
		HEL_CHECK(verifySetupSendRespTail.error());
		co_return;
	}

	verifySetupResp.set_error(managarm::usb::Errors::SUCCESS);

	auto *data = result->value();
	verifySetupResp.set_has_data(data != nullptr);
	if (data) {
		std::vector<uint8_t> newData(data->size());
		memcpy(newData.data(), data->data(), data->size());
		verifySetupResp.set_data(std::move(newData));
	}

	auto [verifySetupSendResp, verifySetupSendRespTail] = co_await helix_ng::exchangeMsgs(
		conversation,
		helix_ng::sendBragiHeadTail(verifySetupResp, frg::stl_allocator{})
	);
	HEL_CHECK(verifySetupSendResp.error());
	HEL_CHECK(verifySetupSendRespTail.error());

	if (!data)
		co_return;

	auto [verifyDataRecvReq] = co_await helix_ng::exchangeMsgs(
		conversation,
		helix_ng::recvInline()
	);
	HEL_CHECK(verifyDataRecvReq.error());

	auto preamble = bragi::read_preamble(verifyDataRecvReq);
	if (preamble.error()) {
		std::cout << "processSetupPacket: error decoding preamble" << std::endl;
		auto [dismiss] = co_await helix_ng::exchangeMsgs(
			conversation, helix_ng::dismiss());
		HEL_CHECK(dismiss.error());
		co_return;
	}

	std::vector<uint8_t> tail(preamble.tail_size());
	auto [recv_tail] = co_await helix_ng::exchangeMsgs(
		conversation,
		helix_ng::recvBuffer(tail.data(), tail.size())
	);
	HEL_CHECK(recv_tail.error());

	auto verifyDataReq = *bragi::parse_head_tail<managarm::usb::VerifySetupPacketDataRequest>(verifyDataRecvReq, tail);
	verifyDataRecvReq.reset();

	if (verifyDataReq.data_size()) {
		data->resize(verifyDataReq.data_size());
		memcpy(data->data(), verifyDataReq.data().data(), verifyDataReq.data_size());
	} else {
		data->resize(verifyDataReq.data_sent());
	}

	result = co_await gen.next();

	managarm::usb::VerifySetupPacketDataResponse verifyDataResp;

	if (result) {
		verifyDataResp.set_error(managarm::usb::Errors::SUCCESS);
	} else {
		verifyDataResp.set_error(toProtocolError(result->error()));
	}

	auto [verifyDataSendResp] = co_await helix_ng::exchangeMsgs(
		conversation,
		helix_ng::sendBragiHeadOnly(verifyDataResp, frg::stl_allocator{})
	);
	HEL_CHECK(verifyDataSendResp.error());
}

async::detached RemoteDeviceController::run() {
	while (true) {
		auto [accept, recv_head] = co_await helix_ng::exchangeMsgs(
			lane_,
			helix_ng::accept(
				helix_ng::recvInline()
			)
		);
		HEL_CHECK(accept.error());
		HEL_CHECK(recv_head.error());

		auto conversation = accept.descriptor();

		auto preamble = bragi::read_preamble(recv_head);

		if (preamble.id() == bragi::message_id<managarm::usb::VerifySetupPacketRequest>) {
			auto req = *bragi::parse_head_only<managarm::usb::VerifySetupPacketRequest>(recv_head);
			co_await processSetupPacket(conversation, req);
		} else {
			std::println(std::cout, "RemoteDeviceController: Illegal request {}", preamble.id());
			auto [dismiss] = co_await helix_ng::exchangeMsgs(
				conversation, helix_ng::dismiss());
			HEL_CHECK(dismiss.error());
		}
	}
}

} // anonymous namespace

Device connect(helix::UniqueLane lane) {
	return Device(std::make_shared<DeviceState>(std::move(lane)));
}

std::unique_ptr<DeviceController> connectDeviceController(helix::UniqueLane lane) {
	return std::make_unique<RemoteDeviceController>(std::move(lane));
}

} // namespace protocols::usb

