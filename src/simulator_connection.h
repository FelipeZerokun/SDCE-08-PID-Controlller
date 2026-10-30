#ifndef SIMULATOR_CONNECTION_H_
#define SIMULATOR_CONNECTION_H_

#include <boost/asio/ip/tcp.hpp>
#include "simulator_session.h"

// Accepts the WebSocket handshake and serves one connection until disconnect.
// Every connection gets a fresh MPC and timing history.
void ServeSimulatorConnection(boost::asio::ip::tcp::socket socket, SessionOptions options = {});

#endif
