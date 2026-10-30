#ifndef SIMULATOR_SESSION_H_
#define SIMULATOR_SESSION_H_
#include "Controller.h"
#include "Metrics.h"
#include <chrono>
#include <optional>
#include <string>
#include <ostream>
struct SessionOptions {
  std::string controller="mpc";
  double target_mph=25;
  std::ostream* log=nullptr;
  unsigned connection=0;
};
class SimulatorSession {
 public:
  explicit SimulatorSession(SessionOptions options = {});
  ~SimulatorSession();
  std::optional<std::string> HandleMessage(const std::string& message);
 private:
  void Reset();
  SessionOptions options_;
  Controller controller_;
  Metrics metrics_;
  unsigned segment_=0;
  double elapsed_=0;
  std::optional<std::chrono::steady_clock::time_point> last_;
};
#endif
