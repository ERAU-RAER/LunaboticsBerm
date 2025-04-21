/**
 * @file    mcvc_cmds.h
 * @brief   API for motor velocity and torque control.
 */

#ifndef MCVC_CMDS_H
#define MCVC_CMDS_H

#include <cstdint>
#include "MotorDriver.h"

/// @addtogroup MotorControl
/// @{

/**
 * @brief Command motor velocity.
 * @param[in] commandedVelocity Target velocity (RPM).
 * @param[in] motor Motor driver instance.
 * @return True if command is valid, false otherwise.
 */
bool CommandVelocity(int32_t commandedVelocity, ClearCore::MotorDriver& motor);

/**
 * @brief Set motor torque limit.
 * @param[in] limit Torque limit (% of max torque).
 * @param[in] motor Motor driver instance.
 * @return True if limit is valid, false otherwise.
 */
bool LimitTorque(double limit, ClearCore::MotorDriver& motor);

/// @}
#endif  // MCVC_CMDS_H