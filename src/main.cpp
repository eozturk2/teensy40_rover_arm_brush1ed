#include <Arduino.h>
#include <SPI.h>
#include "rover_arm.h"
#include "RoverArmMotor.h"
#include "Teensy_PWM.h"
#include "AMT22.h"
#include <limits>
#include <IntervalTimer.h>
#include "TeensyDebug.h"
#include <EEPROM.h>
#include "DMAChannel.h"

#if DEBUG_GDB_STUB == 1
// #pragma GCC optimize("O0")
#endif

// No additional #include statements are needed

#define MIN_FLOAT -std::numeric_limits<float>::max()
#define MAX_FLOAT std::numeric_limits<float>::max()

extern RoverArmMotor Wrist_Roll;
extern RoverArmMotor Wrist_Pitch;
extern RoverArmMotor End_Effector;
extern RoverArmMotor Elbow;
extern RoverArmMotor Shoulder;
extern RoverArmMotor Waist;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
#if DEBUG_GDB_STUB == 1
  debug.begin(SerialUSB1);
  halt_cpu(); // stop on startup; if not, Teensy keeps running and you
#endif
  rover_arm_setup();
}

void loop()
{
#if TEST_LOOP == 1
  rover_arm_loop();
#endif
}
