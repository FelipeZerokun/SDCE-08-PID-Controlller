#include "simulator_connection.h"

#include <boost/asio/io_context.hpp>
#include <charconv>
#include <exception>
#include <iostream>
#include <string_view>
#include <system_error>
#include <utility>
#include <fstream>
#include <filesystem>
#include <cmath>

int main(int argc, char *argv[]) {
  unsigned short port = 4567;
  try {
    SessionOptions options;
    std::string log_path;
    for (int i=1; i<argc; ++i) {
      const std::string arg=argv[i];
      if (arg == "--help") {
        std::cout << "Usage: mpc [port] [--controller pid|mpc|lqr] [--speed mph] [--log new-file.csv]\n";
        return 0;
      }
      if (arg == "--controller" || arg == "--speed" || arg == "--log") {
        if (++i == argc) throw std::invalid_argument("Missing option value");
        if (arg == "--controller") options.controller=argv[i];
        else if (arg == "--log") log_path=argv[i];
        else {
          size_t used=0;
          const std::string value=argv[i];
          options.target_mph=std::stod(value,&used);
          if (used != value.size() || !std::isfinite(options.target_mph) || options.target_mph <= 0)
            throw std::invalid_argument("Speed must be positive and finite");
        }
      } else {
        const auto result=std::from_chars(arg.data(),arg.data()+arg.size(),port);
        if (result.ec != std::errc{} || result.ptr != arg.data()+arg.size() || port == 0)
          throw std::invalid_argument("Unknown option or invalid port; use --help");
      }
    }
    Controller validate(options.controller);
    std::ofstream log;
    if (!log_path.empty()) {
      if (std::filesystem::exists(log_path)) throw std::runtime_error("Log already exists; choose a new filename");
      const auto parent=std::filesystem::path(log_path).parent_path();
      if (!parent.empty()) std::filesystem::create_directories(parent);
      log.open(log_path);
      if (!log) throw std::runtime_error("Cannot open log");
      SimulatorSession::WriteLogHeader(log);
      options.log=&log;
    }
    std::cout << "Controller: " << options.controller << ", target: " << options.target_mph << " mph\n";
    using boost::asio::ip::tcp;
    boost::asio::io_context context;
    tcp::acceptor acceptor(context);
    acceptor.open(tcp::v4());
    acceptor.set_option(tcp::acceptor::reuse_address(true));
    acceptor.bind({boost::asio::ip::make_address("127.0.0.1"), port});
    acceptor.listen();
    std::cout << "Listening on ws://127.0.0.1:" << port
              << " — start the Term 2 simulator in PID mode. Ctrl+C to stop." << std::endl;
    for (;;) {
      auto socket = acceptor.accept();
      try {
        ++options.connection;
        ServeSimulatorConnection(std::move(socket), options);
      } catch (const std::exception &error) {
        std::cerr << "Simulator connection ended: " << error.what() << '\n';
      }
      std::cout << "Waiting for simulator connection..." << std::endl;
    }
  } catch (const std::exception &error) {
    std::cerr << "Simulator server failed: " << error.what() << '\n';
    return 1;
  }
}
