#include "rover_arm.h"
// Drivers.
#include "RoverArmMotor.h"
#include "AMT22.h"
#include <Arduino.h>

// Standard includes.
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <bitset>
#include <limits>

#define MIN_FLOAT -std::numeric_limits<float>::infinity()
#define MAX_FLOAT std::numeric_limits<float>::infinity()
static void attach_all_interrupts();

double setpoint = 0;
int turn = 0;
int limit_set = 0;
int button_counter = 0;
int is_turning = 0;

/*---------------------UART---------------------*/
const int RX_BUFFER_SIZE = 32;
uint8_t rx_data[8]; // 1 byte
char rx_buffer[RX_BUFFER_SIZE];
uint32_t rx_index = 0;
char command_buffer[20];
double param1, param2, param3;

void print_motor(char *msg, void *pMotor)
{
    double current_angle_sw;
    ((RoverArmMotor *)pMotor)->get_current_angle_sw(&current_angle_sw);

    printf("%s setpoint %.2f, angle_sw %.2f, output %.2f, zero_angle_sw %.2f, gear ratio %.2f",
           msg,
           ((RoverArmMotor *)pMotor)->setpoint,
           current_angle_sw,
           ((RoverArmMotor *)pMotor)->output,
           ((RoverArmMotor *)pMotor)->zero_angle_sw,
           ((RoverArmMotor *)pMotor)->gearRatio);
    if (((RoverArmMotor *)pMotor)->encoder_error)
    {
        printf(" (ERROR)\r\n");
    }
    else
    {
        printf("\r\n");
    }
}

/*---------------------WRIST_ROLL_CYTRON---------------------*/
#if TEST_WRIST_ROLL_CYTRON == 1
RoverArmMotor Wrist_Roll(PWM1, DIR1, CS1, CYTRON, 0, 359.99f);
#endif

/*---------------------WRIST_PITCH_CYTRON---------------------*/
#if TEST_WRIST_PITCH_CYTRON == 1
RoverArmMotor Wrist_Pitch(PWM2, DIR2, CS2, CYTRON, 0, 359.99f);
#endif

/*---------------------END_EFFECTOR_CYTRON---------------------*/
#if TEST_END_EFFECTOR_CYTRON == 1
RoverArmMotor End_Effector(&hspi1, CYTRON_PWM_1, CYTRON_DIR_1, AMT22_1, CYTRON, 0, 359.99f);
#endif

/*---------------------ELBOW_SERVO DECLARATIONS---------------------*/
#if TEST_ELBOW_SERVO == 1
RoverArmMotor Elbow(&hspi1, SERVO_PWM_1, dummy_pin, AMT22_1, BLUE_ROBOTICS, 0, 359.99f);
#endif

/*---------------------SHOULDER_SERVO DECLARATIONS---------------------*/
#if TEST_SHOULDER_SERVO == 1
RoverArmMotor Shoulder(&hspi1, SERVO_PWM_1, dummy_pin, AMT22_1, BLUE_ROBOTICS, 0, 359.99f);
#endif

/*---------------------WAIST_SERVO DECLARATIONS---------------------*/
#if TEST_WAIST_SERVO == 1
RoverArmMotor Waist(&hspi1, SERVO_PWM_1, dummy_pin, AMT22_1, BLUE_ROBOTICS, 0, 359.99f);
#endif

void rover_arm_setup(void)
{
    /*---WRIST_ROLL_CYTRON setup---*/
#if TEST_WRIST_ROLL_CYTRON == 1
    Wrist_Roll.wrist_waist = 1;
    Wrist_Roll.setGearRatio(WRIST_ROLL_GEAR_RATIO);
#if SIMULATE_LIMIT_SWITCH == 1
    Wrist_Roll.setGearRatio(1);
#endif
    Wrist_Roll.setAngleLimits(WRIST_ROLL_MIN_ANGLE, WRIST_ROLL_MAX_ANGLE);
    Wrist_Roll.reset_encoder();
    Wrist_Roll.begin(REG_KP_WRIST_ROLL, REG_KI_WRIST_ROLL, REG_KD_WRIST_ROLL);
#if SIMULATE_LIMIT_SWITCH == 1
    Wrist_Roll.stop();
    Wrist_Roll.set_current_as_zero_angle_sw();
    Wrist_Roll.newSetpoint(0.0);
#endif
#endif

    /*---WRIST_PITCH_CYTRON setup---*/
#if TEST_WRIST_PITCH_CYTRON == 1
    Wrist_Pitch.wrist_waist = 0;
    Wrist_Pitch.setGearRatio(WRIST_PITCH_GEAR_RATIO);
    Wrist_Pitch.setAngleLimits(WRIST_PITCH_MIN_ANGLE, WRIST_PITCH_MAX_ANGLE);
    Wrist_Pitch.set_limit_pins(LIMIT_WRIST_PITCH_MAX, LIMIT_WRIST_PITCH_MIN);
    Wrist_Pitch.reset_encoder();
    Wrist_Pitch.begin(REG_KP_WRIST_PITCH, REG_KI_WRIST_PITCH, REG_KD_WRIST_PITCH);
#if SIMULATE_LIMIT_SWITCH == 1
    Wrist_Pitch.stop();
    Wrist_Pitch.set_current_as_zero_angle_sw();
    Wrist_Pitch.newSetpoint(0.0);
#endif
#endif

    /*---END_EFFECTOR_CYTRON setup---*/
#if TEST_END_EFFECTOR_CYTRON == 1
    End_Effector.wrist_waist = 0;
    End_Effector.setAngleLimits(MIN_FLOAT, MAX_FLOAT);
    End_Effector.reset_encoder();
    End_Effector.begin(regKp_end_effector, regKi_end_effector, regKd_end_effector);
    End_Effector.reverse(100);
#if SIMULATE_LIMIT_SWITCH == 1
    End_Effector.stop();
    End_Effector.set_current_as_zero_angle_sw();
    End_Effector.newSetpoint(0.0);
#endif
#endif

    /* ELBOW_SERVO setup */
#if TEST_ELBOW_SERVO == 1
    Elbow.setAngleLimits(0, 120);
    Elbow.reset_encoder();
    Elbow.begin(regKp_elbow, regKi_elbow, regKd_elbow);
#endif
    /*---WAIST_SERVO setup---*/
#if TEST_WAIST_SERVO == 1
    Waist.wrist_waist = 1;
    Waist.setAngleLimits(MIN_FLOAT, MAX_FLOAT);
    Waist.reset_encoder();
    Waist.begin(regKp_waist, regKi_waist, regKd_waist);
#endif

#if SIMULATE_LIMIT_SWITCH == 1

#else
    while (!limit_set)
        ;
#endif
    attach_all_interrupts();
}

void rover_arm_loop()
{
    static unsigned long lastPrint = 0;     // Initialize lastPrint variable
    unsigned long currentMillis = millis(); // get the current "time"

    if (currentMillis - lastPrint >= 100)
    { // If 500ms has passed since the last print operation
#if TEST_ENCODER == 1
        uint16_t encoderData_1 = 0;
        encoderData_1 = getPositionSPI(CS1, 12);
        Serial.printf("encoder 1 gives %d\r\n", encoderData_1);
#endif
#if DEBUG_PRINT_MOTOR == 1
#if TEST_WRIST_ROLL_CYTRON == 1
        print_motor("SP WRIST_ROLL_CYTRON", &Wrist_Roll);
#endif

#if TEST_WRIST_PITCH_CYTRON == 1
        print_motor("SP WRIST_PITCH_CYTRON", &Wrist_Pitch);
#endif

#if TEST_END_EFFECTOR_CYTRON == 1
        print_motor("SP END_EFFECTOR_CYTRON", &End_Effector);
#endif

#if TEST_WAIST_SERVO == 1
        print_motor("SP Waist", &Waist);
#endif

#if TEST_ELBOW_SERVO == 1
        print_motor("SP Elbow", &Elbow);
#endif
#endif
        lastPrint = currentMillis; // Update the lastPrint time
    }
}

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//   // if(!limit_set) {
//   if (GPIO_Pin == B1_Pin && HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) // INT Source is pin A9
//   {
//     is_turning = !is_turning;
// #if TEST_WRIST_ROLL_CYTRON == 1
//     Wrist_Roll.stop();
//     Wrist_Roll.set_current_as_zero_angle_sw();
//     Wrist_Roll.newSetpoint(0.0);
// #endif

// #if TEST_WRIST_PITCH_CYTRON == 1
//     Wrist_Pitch.stop();
//     Wrist_Pitch.set_current_as_zero_angle_sw();
//     Wrist_Pitch.newSetpoint(0.0);
// #endif

// #if TEST_WAIST_SERVO == 1
//     Waist.stop();
//     Waist.set_current_as_zero_angle_sw();
//     Waist.newSetpoint(0.0);
// #endif

// #if TEST_ELBOW_SERVO == 1
//     Elbow.stop();
//     Elbow.set_current_as_zero_angle_sw();
//     Elbow.newSetpoint(0.0);
// #endif

// #if TEST_END_EFFECTOR_CYTRON == 1
//     End_Effector.stop();
//     End_Effector.set_current_as_zero_angle_sw();
//     End_Effector.newSetpoint(0.0);
// #endif

//     limit_set = 1;
//   }

// #if TEST_WRIST_PITCH_CYTRON == 1
//   if (GPIO_Pin == LIMIT_WRIST_PITCH_MIN_Pin && HAL_GPIO_ReadPin(LIMIT_WRIST_PITCH_MIN_GPIO_Port, LIMIT_WRIST_PITCH_MIN_Pin) == GPIO_PIN_RESET)
//   {
//     Wrist_Pitch.stop();
//     Wrist_Pitch.set_current_as_zero_angle_sw();
//   }

//   if (GPIO_Pin == LIMIT_WRIST_PITCH_MAX_Pin && HAL_GPIO_ReadPin(LIMIT_WRIST_PITCH_MAX_GPIO_Port, LIMIT_WRIST_PITCH_MAX_Pin) == GPIO_PIN_RESET)
//   {
//     Wrist_Pitch.stop();
//     Wrist_Pitch.set_max_angle_sw();
//   }
// #endif

//   button_counter++;
//   return;
//   // }
// }

#define DEBOUNCE_DELAY 500 // Delay for 500 ms. Adjust as needed.

volatile unsigned long last_trigger_time_pitch_max = 0;
volatile unsigned long last_trigger_time_pitch_min = 0;
volatile unsigned long last_trigger_time_end_effector_max = 0;
volatile unsigned long last_trigger_time_end_effector_min = 0;

volatile int limit_wrist_pitch_max_activated = 0;
volatile int limit_wrist_pitch_min_activated = 0;
volatile int limit_end_effector_max_activated = 0;
volatile int limit_end_effector_min_activated = 0;

#if TEST_WRIST_PITCH_CYTRON == 1
void limit_wrist_pitch_max_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_pitch_max > DEBOUNCE_DELAY)
    {
        last_trigger_time_pitch_max = now;
        if (digitalRead(LIMIT_WRIST_PITCH_MAX) == LOW)
        {
            limit_wrist_pitch_max_activated = 1;
            Serial.println("Wrist pitch max limit reached");
            Wrist_Pitch.stop();
            Wrist_Pitch.set_current_as_max_angle_sw();
        }
        else
        {
            limit_wrist_pitch_max_activated = 0;
        }
    }
}

void limit_wrist_pitch_min_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_pitch_min > DEBOUNCE_DELAY)
    {
        last_trigger_time_pitch_min = now;
        bool is_low = digitalRead(LIMIT_WRIST_PITCH_MIN) == LOW;
        limit_wrist_pitch_min_activated = is_low;
        if (is_low)
        {
            limit_wrist_pitch_min_activated = 1;
            Serial.println("Wrist pitch min limit reached");
            Wrist_Pitch.stop();
            Wrist_Pitch.set_current_as_zero_angle_sw();
        }
        else
        {
            limit_wrist_pitch_min_activated = 0;
        }
    }
}
#endif

#if TEST_END_EFFECTOR_CYTRON == 1
void limit_end_effector_max_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_end_effector_max > DEBOUNCE_DELAY)
    {
        last_trigger_time_end_effector_max = now;
        if (digitalRead(LIMIT_END_EFFECTOR_MAX) == LOW)
        {
            limit_end_effector_max_activated = 1;
            Serial.println("End effector max limit reached");
            End_Effector.stop();
        }
        else
        {
            limit_end_effector_max_activated = 0;
        }
    }
}

void limit_end_effector_min_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_end_effector_min > DEBOUNCE_DELAY)
    {
        last_trigger_time_end_effector_min = now;
        if (digitalRead(LIMIT_END_EFFECTOR_MIN) == LOW)
        {
            limit_end_effector_min_activated = 1;
            Serial.println("End effector min limit reached");
            End_Effector.stop();
        }
        else
        {
            limit_end_effector_min_activated = 0;
        }
    }
}
#endif

void attach_all_interrupts()
{
#if TEST_WRIST_PITCH_CYTRON == 1
    attachInterrupt(digitalPinToInterrupt(LIMIT_WRIST_PITCH_MAX), limit_wrist_pitch_max_int, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIMIT_WRIST_PITCH_MIN), limit_wrist_pitch_min_int, CHANGE);
#endif

#if TEST_END_EFFECTOR_CYTRON == 1
    attachInterrupt(digitalPinToInterrupt(LIMIT_END_EFFECTOR_MAX), limit_end_effector_max_int, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIMIT_END_EFFECTOR_MIN), limit_end_effector_min_int, CHANGE);
#endif
}
