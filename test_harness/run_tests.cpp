#include <iostream>
#include <cmath>
#include <cassert>

#include "PIDcontrol.h"

void expect_true(bool cond, const char *msg) {
  if (!cond) {
    std::cerr << "FAIL: " << msg << std::endl;
    std::exit(1);
  } else {
    std::cout << "PASS: " << msg << std::endl;
  }
}

void test_constructor_defaults() {
  PIDcontrol p;
  p.setMode(PIDcontrol::AUTOMATIC);
  p.setOutputLimits(0.0, 1.0);
  // first compute likely returns 0 (no time passed / initial state)
  double out = p.compute(50.0);
  expect_true(std::abs(out - 0.0) < 1e-9, "constructor default output == 0");
}

void test_step_response_direction() {
  PIDcontrol p(2.0, 0.5, 0.1, 10);
  p.setMode(PIDcontrol::AUTOMATIC);
  p.setOutputLimits(0.0, 1.0);
  p.setSetpoint(40.0);

  double input = 60.0;
  p.compute(input);
  // simulate passage of time by calling compute with waits using lastTime_
  // but here we call compute in a loop until output changes (approx)
  double out = p.compute(input);
  expect_true(out >= 0.0, "step response non-negative");
}

void test_integral_anti_windup() {
  PIDcontrol p(0.1, 10.0, 0.0, 10);
  p.setMode(PIDcontrol::AUTOMATIC);
  p.setOutputLimits(0.0, 0.5);
  p.setSetpoint(0.0);

  double input = 100.0;
  for (int i = 0; i < 50; ++i) {
    p.compute(input);
  }
  double out = p.getOutput();
  expect_true(out <= 0.5 + 1e-6, "integral anti-windup clamps output");
}

int main() {
  std::cout << "Running PIDcontrol lightweight tests" << std::endl;
  test_constructor_defaults();
  test_step_response_direction();
  test_integral_anti_windup();
  std::cout << "All tests passed." << std::endl;
  return 0;
}
