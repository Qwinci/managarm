#pragma once

#include <helix/ipc.hpp>
#include <smarter.hpp>
#include <vector>

class Ip4Packet;

struct TcpEndpoint {
	friend bool operator<(const TcpEndpoint &l, const TcpEndpoint &r) {
		return std::tie(l.port, l.ipAddress) < std::tie(r.port, r.ipAddress);
	}

	uint32_t ipAddress = 0;
	uint16_t port = 0;
};

struct Tcp4Socket;

struct Tcp4 {
	void feedDatagram(smarter::shared_ptr<const Ip4Packet>);
	bool tryBind(Tcp4Socket *socket, TcpEndpoint ipAddress);
	bool unbind(Tcp4Socket *socket);
	void serveSocket(int flags, helix::UniqueLane lane);

private:
	std::vector<std::pair<TcpEndpoint, Tcp4Socket *>> binds;

};
