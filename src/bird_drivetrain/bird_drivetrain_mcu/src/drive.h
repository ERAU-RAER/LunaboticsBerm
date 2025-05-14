#ifndef DRIVE
#define DRIVE

#include <Arduino.h>
#include <ctype.h>
#include "ClearCore.h"
#include "mcvc_cmds.h"
#include "robot_characteristics.h"

#define baudRate 115200
#define SerialPort ConnectorUsb

void initSpeedHLFB();

void pushWheelSpeedsToJetson();

#endif // DRIVE_H
