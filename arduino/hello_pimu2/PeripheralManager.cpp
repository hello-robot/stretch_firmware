#include "PeripheralManager.h"


void PeripheralManager::fast_actuator_control(bool en)
{
  digitalWrite(LATCH_CTRL, HIGH);
  digitalWrite(LIFT_EN, en);
  digitalWrite(ARM_EN, en);
  digitalWrite(OMNI_0_EN, en);
  digitalWrite(OMNI_1_EN, en);
  digitalWrite(OMNI_2_EN, en);
  digitalWrite(EOA_EN, en);
  digitalWrite(LATCH_CTRL, LOW);
}


void PeripheralManager::rpc_actuator_control(uint8_t actuator, uint8_t enable)
{
  digitalWrite(LATCH_CTRL, HIGH);
  switch (actuator)
  {
  case LIFT_MOTOR:
    digitalWrite(LIFT_EN, enable);
    break;

  case ARM_MOTOR:
    digitalWrite(ARM_EN, enable);
    break;

  case OMNI_0_MOTOR:
    digitalWrite(OMNI_0_EN, enable);
    break;

  case OMNI_1_MOTOR:
    digitalWrite(OMNI_1_EN, enable);
    break;

  case OMNI_2_MOTOR:
    digitalWrite(OMNI_2_EN, enable);
    break;

  case EOA_MOTOR:
    digitalWrite(EOA_EN, enable);
    break;
  
  case ALL:
    fast_actuator_control(enable);

  default:
    break;
  }
  digitalWrite(LATCH_CTRL, LOW);
}

void PeripheralManager::peripheral_sleep_state()
{
    fast_actuator_control(false);
    digitalWrite(DISABLE_5V0, HIGH);
    digitalWrite(DISABLE_20V0, HIGH);
    digitalWrite(RUNSTOP_LED, LOW);
}
void PeripheralManager::peripheral_active_state()
{
    fast_actuator_control(true);
    digitalWrite(DISABLE_5V0, LOW);
    digitalWrite(DISABLE_20V0, LOW);
    digitalWrite(RUNSTOP_LED, HIGH);
}