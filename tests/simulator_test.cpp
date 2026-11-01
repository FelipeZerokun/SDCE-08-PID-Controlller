#include "simulator_connection.h"
#include "simulator_session.h"
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <nlohmann/json.hpp>
#include <cmath>
#include <future>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <map>

void Check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
std::vector<std::string> Columns(const std::string& line) {
  std::vector<std::string> result;
  size_t start=0;
  while(true) {
    const auto end=line.find(',',start);
    result.push_back(line.substr(start,end==std::string::npos?end:end-start));
    if(end==std::string::npos) return result;
    start=end+1;
  }
}
void CheckSteer(const std::string& reply) {
  Check(reply.substr(0, 2) == "42", "Event framing");
  const auto event = nlohmann::json::parse(reply.substr(2));
  Check(event[0] == "steer", "Steer event");
  const double steering = event[1].at("steering_angle");
  const double throttle = event[1].at("throttle");
  Check(std::isfinite(steering) && std::abs(steering) <= 1, "Steering bounds");
  Check(std::isfinite(throttle) && std::abs(throttle) <= 0.3, "Throttle bounds");
}
int main() {
  try {
    SimulatorSession session;
    Check(session.HandleMessage("2probe") == "3probe", "Engine.IO ping");
    Check(session.HandleMessage("40") == "40", "Namespace connect");
    Check(!session.HandleMessage("42[\"other\",{}]"), "Ignore unknown event");
    CheckSteer(*session.HandleMessage("42[\"telemetry\",{\"cte\":\"1.0\",\"speed\":\"20\"}]"));
    Check(session.HandleMessage("42[\"reset\",{}]") == "42[\"manual\",{}]", "Reset");
    CheckSteer(*session.HandleMessage("42[\"telemetry\",{\"cte\":0,\"speed\":20}]"));
    for (const auto& packet : {"42broken", "42[\"telemetry\",{}]",
         "42[\"telemetry\",{\"cte\":\"nan\",\"speed\":20}]",
         "42[\"telemetry\",{\"cte\":\"1junk\",\"speed\":20}]",
         "42[\"telemetry\",{\"cte\":0,\"speed\":-1}]"}) {
      bool rejected = false;
      try { session.HandleMessage(packet); } catch (const std::exception&) { rejected = true; }
      Check(rejected, "Invalid telemetry must be rejected");
    }
    for (const std::string name : {"pid","lqr","mpc"}) {
      std::ostringstream log;
      SimulatorSession::WriteLogHeader(log);
      SessionOptions options; options.controller=name; options.target_mph=15; options.log=&log;
      SimulatorSession configured(options);
      const auto telemetry="42[\"telemetry\",{\"cte\":1,\"speed\":20}]";
      const auto first=configured.HandleMessage(telemetry);
      configured.HandleMessage("42[\"reset\",{}]");
      Check(first==configured.HandleMessage(telemetry),"Reset retains controller configuration");
      Check(log.str().find(name)!=std::string::npos,"Log selected controller");
      std::istringstream csv(log.str()); std::string header,line;
      std::getline(csv,header); std::getline(csv,line);
      const auto keys=Columns(header), values=Columns(line);
      Check(keys.size()==35&&values.size()==keys.size(),"Diagnostic CSV width");
      std::map<std::string,std::string> fields;
      for(size_t k=0;k<keys.size();++k) fields[keys[k]]=values[k];
      Check(keys[0]=="connection"&&keys[9]=="compute_us","Original columns retained");
      Check(fields["diagnostic_schema"]=="1","Diagnostic schema version");
      Check(std::stod(fields["wall_dt_s"])==0.02,"First sample interval");
      Check(std::stod(fields["target_mph"])==15,"Logged target");
      Check(std::abs(std::stod(fields["throttle_raw"])+0.4)<1e-9,"Unclamped throttle demand");
      Check(fields["pause_reset"]=="0","Fresh sample is not a pause");
      Check(!fields["heading_est_rad"].empty(),"State serialized");
      Check((name=="mpc")==fields["raw_steering"].empty(),"Unavailable values blank");
      Check((name=="lqr")==!fields["lqr_error_gain"].empty(),"Controller-specific values");
      Check(nlohmann::json::parse(first->substr(2))[1]["throttle"] < 0,"Configured target speed");
    }
    namespace asio = boost::asio;
    namespace beast = boost::beast;
    using tcp = asio::ip::tcp;
    asio::io_context context;
    tcp::acceptor acceptor(context, {asio::ip::make_address("127.0.0.1"), 0});
    auto server = std::async(std::launch::async, [&] {
      for (const std::string name : {"pid", "lqr", "mpc"}) { SessionOptions o; o.controller=name; ServeSimulatorConnection(acceptor.accept(),o); }
    });
    for (int i = 0; i < 3; ++i) {
      beast::websocket::stream<tcp::socket> client(context);
      client.next_layer().connect(acceptor.local_endpoint());
      client.handshake("127.0.0.1", "/socket.io/?EIO=3&transport=websocket");
      auto exchange = [&](const std::string& packet) {
        client.write(asio::buffer(packet));
        beast::flat_buffer buffer;
        client.read(buffer);
        return beast::buffers_to_string(buffer.data());
      };
      Check(exchange("2") == "3", "Wire ping/pong");
      Check(exchange("40") == "40", "Wire namespace handshake");
      Check(exchange("42broken") == "42[\"manual\",{}]", "Wire invalid input fallback");
      const auto reply = exchange("42[\"telemetry\",{\"cte\":0,\"speed\":20}]");
      CheckSteer(reply);
      Check(nlohmann::json::parse(reply.substr(2))[1]["steering_angle"] == 0.0, "Fresh session after reconnect/error");
      client.close(beast::websocket::close_code::normal);
    }
    server.get();
    std::cout << "Protocol and loopback WebSocket checks passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
