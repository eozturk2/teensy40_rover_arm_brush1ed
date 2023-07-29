// #include <Arduino.h>
#include "RoverArmMotor.h"
#include "Teensy_PWM.h"
#include <Arduino.h>
#include "AMT22.h"
#include "rover_arm.h"
#include <cstdlib>
#include <stdio.h>
#include <inttypes.h>
#include <cmath>
#include "rover_arm.h"

#define BLUE_ROBOTICS_STOP_DUTY_CYCLE 60.0f

// The motor will not move until begin() is called!
/**
 * @brief  Constructor for RoverArmMotor class
 * @param  spi_handle: encoder SPI handle
 * @param  pwm_pin: pin for the PWM
 * @param  dir_pin: only used for Cytron ESC
 * @param  encoder_pin: pin for the encoder
 * @param  esc_type: CYTRON or BLUE_ROBOTICS
 * @param  minimum_angle: minimum angle of the motor
 * @param  maximum_angle: maximum angle of the motor
 * @param  limit_switch_pin: pin for the brake or limit switch
 * @retval None
 */
RoverArmMotor::RoverArmMotor(int pwm_pin, int dir_pin, int encoder_pin, int esc_type,
                             double minimum_angle, double maximum_angle)
{
    _pwm = pwm_pin;
    _dir = dir_pin;
    _encoder = encoder_pin;

    escType = esc_type;
    min_angle = minimum_angle;
    max_angle = maximum_angle;

    _pwm_freq = 400;

    // Zero-initialize all variables
    input = 0;
    output = 0;
    lastAngle = 0;
    useSwAngle = 1;    // default use software angle
    zero_angle_sw = 0; // default no offset
    gear_ratio = 1.0;  // default no multiplier
    wrist_waist = false;

    encoder_error = 0;
    stop_tick = 0;
    fight_gravity = 0;
    inverted = 1.0;

    _brake_pin = -1;
    _limit_pin_max = -1;
    _limit_pin_min = -1;

    angle_full_turn = 360.0f;
}

void RoverArmMotor::begin(double regP, double regI, double regD, double aggP, double aggI, double aggD)
{
    regKp = regP;
    regKi = regI;
    regKd = regD;
    aggKp = aggP;
    aggKi = aggI;
    aggKd = aggD;
    #if DEBUG_ROVER_ARM_MOTOR == 1
        Serial.println("RoverArmMotor::begin()");
        Serial.printf("RoverArmMotor::begin() _brake_pin: %d\r\n", _brake_pin);
    #endif
    /*------------------Set pin modes------------------*/
    pinMode(_pwm, OUTPUT);
    pinMode(_dir, OUTPUT);
    pinMode(_encoder, OUTPUT);
    pinMode(_brake_pin, OUTPUT);
    if (_limit_pin_max != -1 && _limit_pin_min != -1)
    {
        pinMode(_limit_pin_max, INPUT_PULLUP);
        pinMode(_limit_pin_min, INPUT_PULLUP);
    }

    /*------------------Initialize PWM------------------*/
    pwmInstance = new Teensy_PWM(_pwm, _pwm_freq, 0.0f); // 400Hz equals to 2500us period
    #if DEBUG_ROVER_ARM_MOTOR == 1
        Serial.println("RoverArmMotor::begin() 2");
    #endif

    /*------------------Initialize timers------------------*/
    delay(500 * DELAY_FACTOR); // wait for the motor to start up
    this->stop();              // stop the motor
    delay(500 * DELAY_FACTOR); // wait for the motor to start up
    this->stop();              // stop the motor

    #if DEBUG_ROVER_ARM_MOTOR == 1
        Serial.println("_pwm = " + String(_pwm));
        Serial.println("RoverArmMotor::begin() 3");
    #endif

    /*------------------Initialize PID------------------*/
    if (escType == CYTRON)
    {
        #if DEBUG_ROVER_ARM_MOTOR == 1
            Serial.println("RoverArmMotor::begin() 4 CYTRON");
            internalPIDInstance = new PID(PID_DT, 99.0, -99.0, regP, regI, regD);
        #endif
    }
    else if (escType == BLUE_ROBOTICS)
    {
        #if DEBUG_ROVER_ARM_MOTOR == 1
            Serial.println("RoverArmMotor::begin() 4 SERVO");
        #endif
        // Max is actually 400 but this is safer.
        internalPIDInstance = new PID(PID_DT, 300.0, -300.0, regP, regI, regD);
    }

    /*------------------Get setpoint------------------*/
    // Get current location and set it as setpoint. Essential to prevent jerkiness
    // as the microcontroller initializes.
    // adcResult = internalAveragerInstance.reading(analogRead(encoder));
    // after setup, currentAngle is same as setpoint
    delay(100 * DELAY_FACTOR);                       // safety
    int error = get_current_angle_sw(&currentAngle); // fix setpoint not equal to current angle
    if (error == -1)
    {
        encoder_error = 1;
        #if DEBUG_ROVER_ARM_MOTOR == 1
            printf("ERROR: get_current_angle_sw() returned -1 from begin()\r\n");
        #endif
        return;
    }
    #if DEBUG_ROVER_ARM_MOTOR == 1
        Serial.println("Motor current angle: " + String(currentAngle));
    #endif
    setpoint = currentAngle;
    lastAngle = currentAngle;
    #if DEBUG_ROVER_ARM_MOTOR == 1
        Serial.println("RoverArmMotor::begin() 5");
    #endif

    /*------------------Set PID parameters------------------*/
    regKp = regP;
    regKi = regI;
    regKd = regD;

    // if(brake)  engage_brake(); //use brake if there is one
    if (_brake_pin != -1)
        engage_brake(); // use brake if there is one
    
    #if DEBUG_ROVER_ARM_MOTOR == 1
        Serial.println("RoverArmMotor::begin() 6 BEFORE MASTERING");
    #endif

    delay(250 * DELAY_FACTOR); // wait for the motor to start up
    this->stop();              // stop the motor

    /*------------------Mastering------------------*/
#if MASTERING == 1
    #if DEBUG_ROVER_ARM_MOTOR == 1
        Serial.println("RoverArmMotor::begin() 7 Mastering");
    #endif
    delay(250 * DELAY_FACTOR); // wait for the motor to start up
    this->reverse();
    delay(250 * DELAY_FACTOR); // wait for the motor to start up
    this->reverse();
    Serial.println("RoverArmMotor::begin() 8");
#endif

    return;
}

int positive_rezeros = 0;
double real_angle = 0;

// Needs to be called in each loop
void RoverArmMotor::tick()
{
    if (stop_tick)
    {
        this->stop(); // stop the motor
    }
/*------------------Check limit pins------------------*/
// Print limit pins status
#if DEBUG_ROVER_ARM_MOTOR
    Serial.println("limit_pin_max = " + String(_limit_pin_max));
    Serial.println("limit_pin_min = " + String(_limit_pin_min));
    Serial.printf("RoverArmMotor::tick() limit_pin_max = %d\r\n", digitalRead(_limit_pin_max));
    Serial.printf("RoverArmMotor::tick() limit_pin_min = %d\r\n", digitalRead(_limit_pin_min));
#endif
    if (_limit_pin_max != -1 && _limit_pin_min != -1)
    {
        if (digitalRead(_limit_pin_max) == LOW)
        {
            this->stop();
            this->reverse(10);
#if DEBUG_ROVER_ARM_MOTOR
            Serial.println("REVERSING!");
            Serial.println("RoverArmMotor::tick() _limit_pin_max");
#endif
            return;
        }
        if (digitalRead(_limit_pin_min) == LOW)
        {
            this->stop();
            this->forward(10);
#if DEBUG_ROVER_ARM_MOTOR
            Serial.println("ADVANCING!");
            Serial.println("RoverArmMotor::tick() _limit_pin_min");
#endif
            return;
        }
    }

    /*------------------Get current angle------------------*/
    int error = get_current_angle_sw(&currentAngle);
    if (error == -1)
    {
        encoder_error = 1;
        output = 0;
        this->stop(); // stop the motor
#if DEBUG_ROVER_ARM_MOTOR == 1
        printf("ERROR: get_current_angle_sw() returned -1 from tick()\r\n");
#endif
        return;
    }
    else
    {
        encoder_error = 0;
    }

    //------------------Remove jitter------------------//
    // If the change in angle is less than the threshold, return early.
    double diff;
    if (wrist_waist)
    {
        diff = min(abs(currentAngle - setpoint), angle_full_turn - abs(currentAngle - setpoint));
        Serial.printf("diff = %f\r\n", diff);
    }
    else
    {
        diff = abs(currentAngle - setpoint);
    }
    if (diff < (0.8f + fight_gravity * 3.0f))
    {
#if DEBUG_ROVER_ARM_MOTOR == 1
        Serial.println("RoverArmMotor::tick() diff < 0.5");
#endif
        output = 0;
        this->engage_brake();
        this->stop(); // stop the motor
        internalPIDInstance->reset_integral();
        return;
    }

    input = currentAngle; // range is R line
    lastAngle = currentAngle;
    //------------------Compute PID------------------//
    // Retune PID for small errors.
    if (diff > 30.0)
    {
        internalPIDInstance->setPID(aggKp, aggKi, aggKd);
    }
    else
    {
        internalPIDInstance->setPID(regKp, regKi, regKd);
    }
    // Find the shortest from the current position to the setpoint.
    if (wrist_waist)
    {
        forwardDistance = (setpoint > input) ? setpoint - input : (angle_full_turn - input) + setpoint;
        backwardDistance = (setpoint > input) ? (angle_full_turn - setpoint) + input : input - setpoint;
        // GO BACKWARDS CW.
        if (backwardDistance < forwardDistance - 10.0) // handle hysterisis
        {
            if (setpoint > input)
            {
                output = internalPIDInstance->calculate(setpoint, input + angle_full_turn); // buff it 360 to go backwards
#if DEBUG_ROVER_ARM_MOTOR_TICK == 1
                Serial.printf("Case 1, setpoint = %f, input = %f, output = %f\r\n", setpoint, input, output);
#endif
            }
            else
            {
                output = internalPIDInstance->calculate(setpoint, input); // wrapped around so bigger so no need buff 360
#if DEBUG_ROVER_ARM_MOTOR_TICK == 1
                Serial.printf("Case 2, setpoint = %f, input = %f, output = %f\r\n", setpoint, input, output);
#endif
            }
        }
        // GO FORWARDS CCW.
        else
        {
            if (setpoint > input)
            {
                output = internalPIDInstance->calculate(setpoint, input); //  wrapped around so bigger so no need nerf 360
#if DEBUG_ROVER_ARM_MOTOR_TICK == 1
                Serial.printf("Case 3, setpoint = %f, input = %f, output = %f\r\n", setpoint, input, output);
#endif
            }
            else
            {
                output = internalPIDInstance->calculate(setpoint, input - angle_full_turn); // nerf it 360 to go forwards
#if DEBUG_ROVER_ARM_MOTOR_TICK == 1
                Serial.printf("Case 4, setpoint = %f, input = %f, output = %f\r\n", setpoint, input, output);
#endif
            }
        }
    }
    else
    {
        output = internalPIDInstance->calculate(setpoint, input); // return value stored in output
    }

    //------------------Write to motor------------------//
    if (escType == CYTRON)
    {
        //------------------DEADBAND------------------//
        double temp_output = abs(output);
        if (temp_output <= DEADBAND_CYTRON)
        {
            temp_output = 0;
        }
        // Interpret sign of the error signal as the direction pin value
        if (output > 0)
        {
            // B high.
            digitalWrite(_dir, HIGH);
        }
        else
        {
            // A high.
            digitalWrite(_dir, LOW);
        }
        // Write to PWM pin
        pwmInstance->setPWM(_pwm, _pwm_freq, temp_output);
        return;
    }

    // Output range from 1100-1900 us of a 2500 us period
    else if (escType == BLUE_ROBOTICS)
    {
        if (fight_gravity)
        {
            if (output < 0)
            {
                output *= 3.0f;
                output += 20.0f;
                if (diff < 10.0f)
                {
                    output *= 1.0f;
                }
                output = max(output, -220.0f);
            }
            else
            {
                // Light braking.
                output = -25.0f;
            }
        }
        // if (fight_gravity_2) {
        //     if (output < 0)
        //     {
        //         output *= 1.6f;
        //         output += 20.0f;
        //         if (diff < 10.0f)
        //         {
        //             output *= 1.0f;
        //         }
        //         output = max(output, -230.0f);
        //     }
        //     else
        //     {
        //     }
        // }
        this->disengage_brake();
        volatile double output_actual = (1500.0f + output * inverted) * 100.0f / 2500.0f;
        pwmInstance->setPWM(_pwm, _pwm_freq, output_actual);
        return;
    }
    return;
}

int RoverArmMotor::forward(int percentage_speed)
{
    if (percentage_speed < 0 || percentage_speed > 100)
    {
        return -1;
    }
    if (escType == CYTRON)
    {
        // B high.
        digitalWrite(_dir, HIGH);
        pwmInstance->setPWM(_pwm, _pwm_freq, percentage_speed);
        return 0;
    }
    else if (escType == BLUE_ROBOTICS)
    {
        this->disengage_brake();
        double duty_cycle = BLUE_ROBOTICS_STOP_DUTY_CYCLE + (BLUE_ROBOTICS_STOP_DUTY_CYCLE * percentage_speed / 100); // TODEBUG
        pwmInstance->setPWM(_pwm, _pwm_freq, duty_cycle);
        return 0;
    }
    return -1;
}

int RoverArmMotor::reverse(int percentage_speed)
{
    if (percentage_speed < 0 || percentage_speed > 100)
    {
        return -1;
    }
    if (escType == CYTRON)
    {
        // A high.
        digitalWrite(_dir, LOW);
        pwmInstance->setPWM(_pwm, _pwm_freq, percentage_speed);
        return 0;
    }
    else if (escType == BLUE_ROBOTICS)
    {
        this->disengage_brake();
        double duty_cycle = BLUE_ROBOTICS_STOP_DUTY_CYCLE - (BLUE_ROBOTICS_STOP_DUTY_CYCLE * percentage_speed / 100);
        pwmInstance->setPWM(_pwm, _pwm_freq, duty_cycle);
        return 0;
    }
    return -1;
}

void RoverArmMotor::stop()
{
    if (escType == CYTRON)
    {
#if DEBUG_ROVER_ARM_MOTOR == 1
        Serial.println("RoverArmMotor::stop() CYTRON");
#endif
        pwmInstance->setPWM(_pwm, _pwm_freq, 0);
        return;
    }
    else if (escType == BLUE_ROBOTICS)
    {
#if DEBUG_ROVER_ARM_MOTOR == 1
        Serial.println("RoverArmMotor::stop() SERVO");
#endif
        pwmInstance->setPWM(_pwm, _pwm_freq, BLUE_ROBOTICS_STOP_DUTY_CYCLE);
        return;
    }
    return;
}

// Useless at the moment.
void RoverArmMotor::set_PID_params(double regP, double regI, double regD)
{
    regKp = regP;
    regKi = regI;
    regKd = regD;

    return;
}

bool RoverArmMotor::setMultiplierBool(bool mult, double ratio)
{
    wrist_waist = mult;
    gear_ratio = ratio;
    // a bit redundant but just a sanity check of a second getter method
    if (getRatio() == ratio)
        return true;
    return false;
}

// For display purposes
double RoverArmMotor::get_setpoint()
{
    return setpoint / gear_ratio;
}

// Remove gear_ration burden from user.
bool RoverArmMotor::new_setpoint(double angle)
{
    #if DEBUG_ROVER_ARM_MOTOR == 1
        Serial.printf("RoverArmMotor::new_setpoint() angle = %f\r\n", angle);
    #endif
    double temp_setpoint = angle * gear_ratio;
    // Extra step for wrist_waist.
    if (wrist_waist)
    {
        temp_setpoint = std::fmod(temp_setpoint, angle_full_turn);
        if (temp_setpoint < 0)
        {
            temp_setpoint += angle_full_turn;
        }
    }
    if (temp_setpoint >= min_angle && temp_setpoint <= max_angle)
    {
        setpoint = temp_setpoint;
        return true;
    }
    else
    {
        return false;
    }
}

void RoverArmMotor::set_gear_ratio(double ratio)
{
    gear_ratio = ratio;
    angle_full_turn = 360.0f * gear_ratio;
    return;
}

void RoverArmMotor::setAngleLimits(double lowest, double highest)
{
    min_angle = lowest * gear_ratio;
    max_angle = highest * gear_ratio;
    return;
}

void RoverArmMotor::set_zero_angle()
{
    setZeroSPI(_encoder); // timer not used, so nullptr
    return;
}

void RoverArmMotor::reset_encoder()
{
    resetAMT22(_encoder); // timer not used, so nullptr
    return;
}

void RoverArmMotor::set_current_as_zero_angle_sw()
{
    this->get_current_angle_multi(&zero_angle_sw);
    return;
}

void RoverArmMotor::set_current_as_zero_angle_sw(double angle)
{
    zero_angle_sw = angle;
    return;
}

void RoverArmMotor::set_current_as_max_angle_sw()
{
    set_current_as_angle_sw(max_angle);
}

void RoverArmMotor::set_current_as_angle_sw(double angle)
{
    double temp = 0;
    this->get_current_angle_multi(&temp);
    zero_angle_sw = temp - angle;
}

uint32_t RoverArmMotor::get_turns_encoder()
{
    uint32_t turns = get_turns_AMT22(_encoder, 12);
    return turns;
}

void RoverArmMotor::disengage_brake()
{
    if (_brake_pin != -1)
    {
        digitalWrite(_brake_pin, HIGH);
        return;
    }
    return;
}

void RoverArmMotor::engage_brake()
{
    if (_brake_pin != -1)
    {
        digitalWrite(_brake_pin, LOW);
        return;
    }
    return;
}

double RoverArmMotor::get_current_angle()
{
    uint16_t encoderData = getPositionSPI(_encoder, 12);
    currentAngle = mapFloat((float)encoderData, MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f);
    return currentAngle;
}

int RoverArmMotor::get_current_angle_multi(double *angle)
{
    int16_t result_arr[2];
    int error = getTurnCounterSPI(result_arr, _encoder, 12); // timer not used, so nullptr
    if (error == -1)
    {
#if DEBUG_ROVER_ARM_MOTOR == 1
        printf("ERROR: getTurnCounterSPI() returned -1 from get_current_angle_multi()\r\n");
#endif
        return -1;
    }

    _angle_raw = mapFloat((float)result_arr[0], MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f);
    _turns = result_arr[1];
    *angle = _angle_raw + 360 * _turns;

    return 0;
}

int RoverArmMotor::get_current_angle_sw(double *angle)
{
    int error = get_current_angle_multi(&current_angle_multi);
    if (error == -1)
    {
        encoder_error = 1;
#if DEBUG_ROVER_ARM_MOTOR == 1
        printf("ERROR: get_current_angle_multi() returned -1 from get_current_angle_sw()\r\n");
#endif
        return -1;
    }
    encoder_error = 0;
    double diff = current_angle_multi - zero_angle_sw;
    if (wrist_waist) // TODO optimize
    {
        double temp;
        temp = std::fmod(diff, (360 * gear_ratio));
        if (temp < 0)
        {
            temp += (360 * gear_ratio);
        }
        *angle = temp;
    }
    else
    {
        *angle = diff;
    }
    return 0; // return 0 on success
}

double RoverArmMotor::getCurrentOutput()
{
    return output;
}

double RoverArmMotor::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    double result = ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);

    return result;
}

double RoverArmMotor::getRatio()
{
    return gear_ratio;
}

int RoverArmMotor::get_turn_count()
{
    int16_t result_arr[2];
    getTurnCounterSPI(result_arr, _encoder, 12); // timer not used, so nullptr
    return result_arr[1];
}

void RoverArmMotor::set_limit_pins(int limit_pin_max, int limit_pin_min)
{
    _limit_pin_max = limit_pin_max;
    _limit_pin_min = limit_pin_min;
    pinMode(_limit_pin_max, INPUT_PULLUP);
    pinMode(_limit_pin_min, INPUT_PULLUP);
}
void RoverArmMotor::set_safety_pins(int brake_pin, int limit_pin_max, int limit_pin_min)
{
    _brake_pin = brake_pin;
    _limit_pin_max = limit_pin_max;
    _limit_pin_min = limit_pin_min;
    pinMode(_brake_pin, OUTPUT);
    pinMode(_limit_pin_max, INPUT_PULLUP);
    pinMode(_limit_pin_min, INPUT_PULLUP);
}