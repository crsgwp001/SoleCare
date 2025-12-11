// ...existing code...
#include "PIDcontrol.h"

PIDcontrol::PIDcontrol(double kp, double ki, double kd, unsigned long sampleMs)
    : kp_(kp), ki_(ki), kd_(kd), outMin_(0.0), outMax_(1.0), setpoint_(0.0),
      output_(0.0), integral_(0.0), lastInput_(0.0), lastErr_(0.0),
      sampleMs_(sampleMs), lastTime_(0), mode_(MANUAL) {}

void PIDcontrol::setTunings(double kp, double ki, double kd) {
  if (kp < 0 || ki < 0 || kd < 0) return;
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDcontrol::setSampleTime(unsigned long sampleMs) {
  if (sampleMs > 0) sampleMs_ = sampleMs;
}

void PIDcontrol::setOutputLimits(double minOut, double maxOut) {
  if (minOut >= maxOut) return;
  outMin_ = minOut;
  outMax_ = maxOut;
  output_ = clamp(output_, outMin_, outMax_);
  integral_ = clamp(integral_, outMin_, outMax_);
}

void PIDcontrol::setMode(Mode mode) {
  if (mode == AUTOMATIC && mode_ == MANUAL) reset();
  mode_ = mode;
}

void PIDcontrol::setSetpoint(double sp) { setpoint_ = sp; }

double PIDcontrol::compute(double input) {
  if (mode_ == MANUAL) return output_;
  unsigned long now = millis();
  unsigned long dt = now - lastTime_;
  if (lastTime_ == 0) {
    lastTime_ = now;
    lastInput_ = input;
    lastErr_ = setpoint_ - input;
    return output_;
  }
  if (dt < sampleMs_) return output_;

  double error = setpoint_ - input;

  // Proportional term
  double P = kp_ * error;

  // Integral term (trapezoidal approximation)
  integral_ += 0.5 * (error + lastErr_) * (ki_ * (dt / 1000.0));
  integral_ = clamp(integral_, outMin_, outMax_); // anti-windup

  // Derivative on measurement to reduce derivative kick
  double dInput = (input - lastInput_) / (dt / 1000.0);
  double D = -kd_ * dInput;

  // Output: start at midpoint, then apply PID corrections
  double midpoint = (outMin_ + outMax_) / 2.0;
  double range = outMax_ - outMin_;
  double out = midpoint + (P + integral_ + D) * range / 2.0;
  out = clamp(out, outMin_, outMax_);

  // Save state
  output_ = out;
  lastErr_ = error;
  lastInput_ = input;
  lastTime_ = now;
  return output_;
}

double PIDcontrol::getOutput() const { return output_; }

void PIDcontrol::reset() {
  integral_ = 0.0;
  lastInput_ = 0.0;
  lastErr_ = 0.0;
  output_ = clamp(output_, outMin_, outMax_);
  lastTime_ = millis();
}

double PIDcontrol::clamp(double v, double lo, double hi) const {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}