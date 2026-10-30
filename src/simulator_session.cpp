#include "simulator_session.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace {
using nlohmann::json;
constexpr const char* manual = "42[\"manual\",{}]";
double Number(const json& value) {
  double result;
  if (value.is_string()) {
    const auto text = value.get<std::string>();
    size_t used = 0;
    result = std::stod(text, &used);
    if (used != text.size()) throw std::invalid_argument("Trailing text in telemetry number");
  } else if (value.is_number()) {
    result = value.get<double>();
  } else {
    throw std::invalid_argument("Expected numeric telemetry");
  }
  if (!std::isfinite(result)) throw std::invalid_argument("Nonfinite telemetry");
  return result;
}
}
SimulatorSession::SimulatorSession(SessionOptions options) : options_(options), controller_(options.controller) {
  if (!std::isfinite(options.target_mph) || options.target_mph <= 0)
    throw std::invalid_argument("Target speed must be positive and finite");
}
SimulatorSession::~SimulatorSession() {
  if (metrics_.count) {
    std::cout << options_.controller << " summary (seconds,RMS CTE,max CTE,RMS steering rate,saturation %,mean compute us): ";
    metrics_.Write(std::cout); std::cout << std::endl;
  }
}
void SimulatorSession::Reset() {
  controller_.Reset(); last_.reset(); metrics_={}; elapsed_=0; ++segment_;
}
std::optional<std::string> SimulatorSession::HandleMessage(const std::string& message) {
  // Same Engine.IO / Socket.IO subset used by the EKF simulator adapter.
  if (!message.empty() && message[0] == '2') return "3" + message.substr(1);
  if (message == "40") return "40";
  if (message.compare(0, 2, "42") != 0) return std::nullopt;
  try {
    const auto event = json::parse(message.substr(2));
    if (event.is_null()) { Reset(); return manual; }
    if (!event.is_array() || event.size() != 2 || !event[0].is_string())
      throw std::invalid_argument("Expected Socket.IO [event, payload]");
    const auto name = event[0].get<std::string>();
    if (name == "reset") { Reset(); return manual; }
    if (name != "telemetry") return std::nullopt;
    if (event[1].is_null()) { Reset(); return manual; }
    const double cte = Number(event[1].at("cte"));
    const double mph = Number(event[1].at("speed"));
    const auto now = std::chrono::steady_clock::now();
    const double dt = last_ ? std::chrono::duration<double>(now - *last_).count() : 0.02;
    const auto compute_start = std::chrono::steady_clock::now();
    const double steering = controller_.Update(cte, mph * 0.44704, dt);
    const double compute_us = std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-compute_start).count();
    last_ = now;
    // Identical longitudinal controller for every lateral controller.
    const double throttle = std::clamp(0.08 * (options_.target_mph - mph), -0.3, 0.3);
    // A long pause starts a fresh measurement segment and is excluded from metrics.
    if (dt > 0.5) { metrics_={}; elapsed_=0; ++segment_; }
    const double sample_dt = dt > 0.5 ? 0.02 : dt;
    elapsed_ += sample_dt;
    metrics_.Add(cte,steering,sample_dt,compute_us);
    if (options_.log) {
      *options_.log << options_.connection << ',' << segment_ << ',' << options_.controller << ','
          << elapsed_ << ',' << sample_dt << ',' << cte << ',' << mph << ',' << steering << ',' << throttle << ',' << compute_us << '\n';
      options_.log->flush();
      if (!*options_.log) throw std::runtime_error("Telemetry log write failed");
    }
    return "42" + json::array({"steer", {{"steering_angle", steering}, {"throttle", throttle}}}).dump();
  } catch (...) {
    Reset();
    throw;
  }
}
