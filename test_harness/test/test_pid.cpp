#include <unity.h>
#include "PIDcontrol.h"

void test_pid_basic() {
  PIDcontrol p(1.0, 0.1, 0.0, 10);
  p.setMode(PIDcontrol::AUTOMATIC);
  p.setOutputLimits(0.0, 1.0);
  p.setSetpoint(40.0);
  double out = p.compute(60.0);
  TEST_ASSERT_TRUE(out >= 0.0);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_pid_basic);
  UNITY_END();
  return 0;
}
