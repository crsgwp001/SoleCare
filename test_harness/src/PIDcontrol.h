#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <chrono>
#include <cstdint>
static inline unsigned long millis() {
  using namespace std::chrono;
  static const auto start = steady_clock::now();
  return static_cast<unsigned long>(duration_cast<milliseconds>(steady_clock::now() - start).count());
}
#endif

class PIDcontrol {
public:
  enum Mode { MANUAL = 0, AUTOMATIC = 1 };

  PIDcontrol(double kp = 1.0, double ki = 0.0, double kd = 0.0, unsigned long sampleMs = 200);

  void setTunings(double kp, double ki, double kd);
  void setSampleTime(unsigned long sampleMs);
  void setOutputLimits(double minOut, double maxOut);
  void setMode(Mode mode);
  void setSetpoint(double sp);
  double compute(double input);
  double getOutput() const;
  void reset();

private:
  double kp_, ki_, kd_;
  double outMin_, outMax_;
  double setpoint_;
  double output_;
  double integral_;
  double lastInput_;
  double lastErr_;
  unsigned long sampleMs_;
  unsigned long lastTime_;
  Mode mode_;

  double clamp(double v, double lo, double hi) const;
};
