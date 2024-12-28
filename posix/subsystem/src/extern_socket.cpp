#include "extern_socket.hpp"

#include "fs.bragi.hpp"
#include "protocols/fs/client.hpp"

namespace {
struct Socket : File {
	Socket(helix::UniqueLane control, helix::UniqueLane sockLane)
	: File{StructName::get("extern-socket")},
		_control{std::move(control)}, _file{std::move(sockLane)} { }

	async::result<frg::expected<Error, PollWaitResult>>
	pollWait(Process *, uint64_t sequence, int mask,
			async::cancellation_token cancellation) override {
		auto resultOrError = co_await _file.pollWait(sequence, mask, cancellation);
		assert(resultOrError);
		co_return resultOrError.value();
	}

	async::result<frg::expected<Error, PollStatusResult>>
	pollStatus(Process *) override {
		auto resultOrError = co_await _file.pollStatus();
		assert(resultOrError);
		co_return resultOrError.value();
	}

	async::result<protocols::fs::Error> listen() override {
		managarm::fs::CntRequest req;
		req.set_req_type(managarm::fs::CntReqType::PT_LISTEN);

		auto ser = req.SerializeAsString();
		uint8_t buffer[128];

		auto [offer, send_req, recv_resp] =
			co_await helix_ng::exchangeMsgs(
				getPassthroughLane(),
				helix_ng::offer(
					helix_ng::sendBuffer(ser.data(), ser.size()),
					helix_ng::recvBuffer(buffer, 128)
				)
			);

		HEL_CHECK(offer.error());
		HEL_CHECK(send_req.error());
		HEL_CHECK(recv_resp.error());

		managarm::fs::SvrResponse resp;
		resp.ParseFromArray(buffer, recv_resp.actualLength());

		if(resp.error() == managarm::fs::Errors::ILLEGAL_OPERATION_TARGET) {
			co_return protocols::fs::Error::illegalOperationTarget;
		}else {
			assert(resp.error() == managarm::fs::Errors::SUCCESS);
			co_return protocols::fs::Error::none;
		}
	}

	async::result<frg::expected<Error, AcceptResult>> accept(Process *) override {
		managarm::fs::CntRequest req;
		req.set_req_type(managarm::fs::CntReqType::PT_ACCEPT);

		auto req_data = req.SerializeAsString();
		char buffer[128];

		auto [offer, send_req, recv_resp, recv_lane, recv_ctrl] = co_await helix_ng::exchangeMsgs(
			getPassthroughLane(),
			helix_ng::offer(
				helix_ng::sendBuffer(req_data.data(), req_data.size()),
				helix_ng::recvBuffer(buffer, sizeof(buffer)),
				helix_ng::pullDescriptor(),
				helix_ng::pullDescriptor()
			)
		);
		HEL_CHECK(offer.error());
		HEL_CHECK(send_req.error());
		HEL_CHECK(recv_resp.error());

		managarm::fs::SvrResponse resp;
		resp.ParseFromArray(buffer, recv_resp.actualLength());

		switch(resp.error()) {
		case managarm::fs::Errors::ILLEGAL_OPERATION_TARGET:
			co_return Error::illegalOperationTarget;
		case managarm::fs::Errors::WOULD_BLOCK:
			co_return Error::wouldBlock;
		default:
			HEL_CHECK(recv_lane.error());
			HEL_CHECK(recv_ctrl.error());
			assert(resp.error() == managarm::fs::Errors::SUCCESS);
		}

		auto file = smarter::make_shared<Socket>(recv_ctrl.descriptor(), recv_lane.descriptor());
		file->setupWeakFile(file);
		co_return File::constructHandle(file);
	}

	void handleClose() override {
		// Close the control lane to inform the server that we closed the file.
		_control = helix::UniqueLane{};
	}

	helix::BorrowedDescriptor getPassthroughLane() override {
		return _file.getLane();
	}

private:
	helix::UniqueLane _control;
	protocols::fs::File _file;
};
}

namespace extern_socket {

async::result<smarter::shared_ptr<File, FileHandle>> createSocket(helix::BorrowedLane lane,
		int domain, int type, int proto, int flags) {
	managarm::fs::CntRequest req;
	req.set_req_type(managarm::fs::CntReqType::CREATE_SOCKET);
	req.set_domain(domain);
	req.set_type(type);
	req.set_protocol(proto);
	req.set_flags(flags);

	auto req_data = req.SerializeAsString();
	char buffer[128];

	auto [offer, send_req, recv_resp, recv_lane, recv_ctrl] = co_await helix_ng::exchangeMsgs(
		lane,
		helix_ng::offer(
			helix_ng::sendBuffer(req_data.data(), req_data.size()),
			helix_ng::recvBuffer(buffer, sizeof(buffer)),
			helix_ng::pullDescriptor(),
			helix_ng::pullDescriptor()
		)
	);
	HEL_CHECK(offer.error());
	HEL_CHECK(send_req.error());
	HEL_CHECK(recv_resp.error());
	HEL_CHECK(recv_lane.error());
	HEL_CHECK(recv_ctrl.error());

	managarm::fs::SvrResponse resp;
	resp.ParseFromArray(buffer, recv_resp.actualLength());
	assert(resp.error() == managarm::fs::Errors::SUCCESS);

	auto file = smarter::make_shared<Socket>(recv_ctrl.descriptor(), recv_lane.descriptor());
	file->setupWeakFile(file);
	co_return File::constructHandle(file);
}

} // namespace extern_socket
