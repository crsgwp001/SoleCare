// Unit tests for PIDcontrol
#include <Arduino.h>
#include <unity.h>
#include "PIDcontrol.h"

static void test_constructor_defaults(void) {
  PIDcontrol p;
  p.setMode(PIDcontrol::AUTOMATIC);
  p.setOutputLimits(0.0, 1.0);
  // With no time elapsed, compute should return default output (0)
  double out = p.compute(50.0);
  TEST_ASSERT_EQUAL_FLOAT(0.0, out);
}

static void test_step_response_direction(void) {
  PIDcontrol p(2.0, 0.5, 0.1, 10);
  p.setMode(PIDcontrol::AUTOMATIC);
  p.setOutputLimits(0.0, 1.0);
  p.setSetpoint(40.0);

  // start from high humidity (wet)
  double input = 60.0;
  // First compute initializes internal timers, subsequent computes after sample time
  p.compute(input);
  delay(20);
  double out1 = p.compute(input);
  // output should be > 0 because input > setpoint and controller will push positive output
  TEST_ASSERT(out1 > 0.0);
}

static void test_integral_anti_windup(void) {
  PIDcontrol p(0.1, 10.0, 0.0, 10); // high Ki to exercise integral
  p.setMode(PIDcontrol::AUTOMATIC);
  p.setOutputLimits(0.0, 0.5);
  p.setSetpoint(0.0);

  // Provide large positive error repeatedly to try to wind up integral
  double input = 100.0;
  for (int i = 0; i < 50; ++i) {
    p.compute(input);
    delay(10);
  }
  double out = p.getOutput();
  // Output must be clamped to configured max (0.5)
  TEST_ASSERT_TRUE(out <= 0.5 + 1e-6);
}

void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_constructor_defaults);
  RUN_TEST(test_step_response_direction);
  RUN_TEST(test_integral_anti_windup);
  UNITY_END();
}

void loop() {
  // nothing
}
