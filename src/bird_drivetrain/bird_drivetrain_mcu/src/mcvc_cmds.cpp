/**
 * @file    mcvc_cmds.cpp
 * @brief   Velocity and torque commands for a bipolar PWM motor.
 */

#include "mcvc_cmds.h"
#include "ClearCore.h"
#include <Arduino.h>

//----------------------------------------------------------------------------//
// Configuration parameters
//----------------------------------------------------------------------------//

/** Max velocity (device units). */
double maxVelocity = 3100.0;

/** Torque limit range (0–100%). */
double torqueLimit = 100.0;
double torqueLimitAlternate = 10.0;

/** Dead-band width around neutral duty (%). */
double pwmDeadBand = 2.0;

//----------------------------------------------------------------------------//
/**
 * @brief Command motor velocity.
 * @param commandedVelocity Desired velocity (device units).
 * @param motor Motor driver instance.
 * @return True if command is valid, false otherwise.
 */
bool CommandVelocity(int32_t commandedVelocity, ClearCore::MotorDriver& motor) {
    if (abs(commandedVelocity) > maxVelocity) {
        return false;
    }

    double halfSpan = (255.0 / 2.0) - (pwmDeadBand / 100.0 * 255.0);
    double scale = halfSpan / maxVelocity;
    double duty = 128.0;

    if (commandedVelocity < 0) {
        duty = 128.0 - (pwmDeadBand / 100.0 * 255.0) + (commandedVelocity * scale);
    }
    else if (commandedVelocity > 0) {
        duty = 128.0 + (pwmDeadBand / 100.0 * 255.0) + (commandedVelocity * scale);
    }

    motor.MotorInBDuty(duty);
    return true;
}

//----------------------------------------------------------------------------//
/**
 * @brief Limit motor torque.
 * @param limit Desired torque limit (0–100%).
 * @param motor Motor driver instance.
 * @return True if limit is valid, false otherwise.
 */
bool LimitTorque(double limit, ClearCore::MotorDriver& motor) {
    if (limit < torqueLimitAlternate || limit > torqueLimit) {
        return false;
    }

    double rangePct = torqueLimit - torqueLimitAlternate;
    double scale = 255.0 / rangePct;
    uint8_t duty = static_cast<uint8_t>((torqueLimit - limit) * scale);

    motor.MotorInADuty(duty);
    return true;
}
/** 
 * @brief Gets current speed in RPM
 * @param motor Motor driver instance.
 * @return current speed in RPM
 */
// double GetSpeed(ClearCore::MotorDriver& motor) {
//     double currentSpeed = EncoderIn.Velocity(); //look into this and motor.statusreg.bit
//     return currentSpeed;
// }

/**
* @brief Sets input A low in CPM_A
*/
void useFullTorque(ClearCore::MotorDriver& motor) {
    // Set the input A low
    if(motor.MotorInAState()) {
        motor.MotorInAState(false);
    }
}