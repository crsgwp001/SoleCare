// ...existing code...
#pragma once

#include <Arduino.h>

class PIDcontrol {
public:
  enum Mode { MANUAL = 0, AUTOMATIC = 1 };

  PIDcontrol(double kp = 1.0, double ki = 0.0, double kd = 0.0, unsigned long sampleMs = 200);

  // Main API
  void setTunings(double kp, double ki, double kd);
  void setSampleTime(unsigned long sampleMs);
  void setOutputLimits(double minOut, double maxOut);
  void setMode(Mode mode);
  void setSetpoint(double sp);
  double compute(double input); // call regularly; returns current output (normalized)
  double getOutput() const;
  double getSetpoint() const { return setpoint_; }  // Get current setpoint for logging

  // Utility
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

  // internal helpers
  double clamp(double v, double lo, double hi) const;
};