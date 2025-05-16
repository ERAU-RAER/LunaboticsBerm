// #include <Arduino.h>
// #include <ctype.h>
// #include "ClearCore.h"
// #include "mcvc_cmds.h"
// #include "robot_characteristics.h"

// #include "drive.h"

// #define baudRate 115200
// #define SerialPort ConnectorUsb

// #define outputPin ConnectorIO1

// #define MAX_ANGLE 180
// #define MIN_ANGLE 0
// #define MAX_MS 2500
// #define MIN_MS 500

// MotorDriver* motors[] = { &ConnectorM0, &ConnectorM1, &ConnectorM2, &ConnectorM3 };
// bool motorDirection[] = {1, 1, 0, 0}; // 1 : non-inverted - 0 : inverted

// void handleCommand(const char* input) {
//   // sanity: first char is command letter
//   char cmd = input[0];
//   if (cmd >= 'A' && cmd <= 'Z') cmd = cmd - 'A' + 'a';
//   if (cmd < 'a' || cmd > 'z') {
//       SerialPort.SendLine("ERR: invalid command");
//       return;
//   }

//   // parse motor number (can be multi‐digit)
//   int pos = 1, motorNum = 0;
//   if (!isdigit(input[pos])) {
//       SerialPort.SendLine("ERR: bad motor #");
//       return;
//   }
//   while (isdigit(input[pos])) {
//       motorNum = motorNum * 10 + (input[pos++] - '0');
//   }
//   if (motorNum < 0 || motorNum > 3) {
//       SerialPort.SendLine("ERR: bad motor #");
//       return;
//   }

//   // skip spaces, then parse argument
//   while (input[pos] == ' ') pos++;
//   int32_t arg = atoi(&input[pos]);

//   switch (cmd) {
//     case 'v':
//       if (!motorDirection[motorNum]) {
//         arg = -arg;
//       }
//       CommandVelocity(arg, *motors[motorNum]);
      
//       {
//         char msg[48];
//         snprintf(msg, sizeof(msg), "OK: velocity set to %ld on v%d", (long)arg, motorNum);
//         SerialPort.SendLine(msg);
//       }
      
//       break;
//     default:
//       SerialPort.SendLine("ERR: unknown cmd");
//       break;
//   }
// }

// int main() {

//   // outputPin.Mode(Connector::OUTPUT_PWM);
//   // float ms_amount = map(90, MIN_ANGLE, MAX_ANGLE, MIN_MS, MAX_MS);
//   // outputPin.writeMicroseconds(ms_amount);

//   // outputPin.PwmDuty(127);
//   // delay(1000);
//   // outputPin.PwmDuty(0);
//   // delay(1000);
//   // outputPin.PwmDuty(255);

//   // Sets all motor connectors to the correct mode for Follow Digital
//   // Velocity, Bipolar PWM mode.
//   MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,Connector::CPM_MODE_A_PWM_B_PWM);

//   SerialPort.Mode(Connector::USB_CDC);
//   SerialPort.Speed(baudRate);
//   uint32_t timeout = 5000;
//   uint32_t startTime = Milliseconds();
//   SerialPort.PortOpen();
//   while (!SerialPort && Milliseconds() - startTime < timeout) {
//       continue;
//   }

//   // Welcome message
//   SerialPort.SendLine("Good Morning, B.I.R.D. ...");
//   SerialPort.SendLine("╰╮╰╮╰╮");
//   SerialPort.SendLine("╭━━━━━━━╮╱ ");
//   SerialPort.SendLine("╰━━━━━━━╯╱ ");
//   SerialPort.SendLine("┃╭╭╮┏┏┏┏┣━╮");
//   SerialPort.SendLine("┃┃┃┃┣┣┣┣┃╱┃ ");
//   SerialPort.SendLine("┃╰╰╯┃┃┗┗┣━╯ ");
//   SerialPort.SendLine("╰━━━━━━━╯ ");

//   // Enables the motor
//   motors[0] -> EnableRequest(true);
//   motors[1] -> EnableRequest(true);
//   motors[2] -> EnableRequest(true);
//   motors[3] -> EnableRequest(true);

//   for (auto m : motors) {
//     m->MotorInADuty(0.0);   // 0% → use primary (100%) torque limits
//   }

//   LimitTorque(100, *motors[0]);
//   LimitTorque(100, *motors[1]);
//   LimitTorque(100, *motors[2]);
//   LimitTorque(100, *motors[3]);

//   SerialPort.SendLine("Motors Enabled");

//   // Waits for 5 seconds to allow the motor to come up to speed
//   SerialPort.SendLine("Waiting for motor to reach speed...");
//   startTime = Milliseconds();
//   while (Milliseconds() - startTime < timeout) {
//       continue;
//   }
//   SerialPort.SendLine("Motors Ready");

//   const int IN_BUFFER_LEN = 64;
//   char  input[IN_BUFFER_LEN+1];
//   int   idx      = 0;
//   bool  overflow = false;

//   while (true) {
//     // consume all available chars
//     while (SerialPort.CharPeek() != -1) {
//       char c = (char)SerialPort.CharGet();
//       Delay_ms(1);
//       if (c == '\r') {
//         continue;
//       } else if (c == '\n') {
//         input[idx] = '\0';
//         if (overflow) {
//           SerialPort.SendLine("ERR: buffer overrun");
//         } else if (idx > 0) {
//           handleCommand(input);
//         }
//         idx      = 0;
//         overflow = false;
//       } else {
//         if (idx < IN_BUFFER_LEN) {
//           input[idx++] = c;
//         } else {
//           overflow = true;
//         }
//       }
//     }
//     pushWheelSpeedsToJetson();
//   }
// }

/*
 * Title: WritePwmOutput
 *
 * Objective:
 *    This example demonstrates how to write a digital PWM signal to a ClearCore
 *    digital output.
 *
 * Description:
 *    This example sets the defined pin as an output then writes a series of
 *    PWM signals with varying duty cycles to the output.
 *
 * Requirements:
 * ** Connect a device that takes in a PWM signal to IO-1.
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 *
 * 
 * Copyright (c) 2020 Teknic Inc. This work is free to use, copy and distribute under the terms of
 * the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */

 #include "ClearCore.h"
 #include "Arduino.h"
//  #include "ledc.h"

 // Specify which output pin to write digital PWM to.
 // PWM-capable pins: IO-0 through IO-5.
 // Note: IO-4 and IO-5 are capable of bi-directional and higher-current PWM
 //       output using an H-Bridge. See the WriteHBridgeOutput example.
 #define outputPin ConnectorIO3
 
#define MAX_ANGLE 180
#define MIN_ANGLE 45
#define MAX_MS 2.5
#define MIN_MS 1
 
 int main() {
     // Set up the output pin for PWM output mode.
    outputPin.Mode(Connector::OUTPUT_DIGITAL);

    // Begin a 100ms on/200ms off pulse on IO-1's output that will complete
    // 20 cycles and prevent further code execution until the cycles are
    // complete
    int angle = 90; // Example angle value
    float ms_off = map(angle, MIN_ANGLE, MAX_ANGLE, MIN_MS, MAX_MS);
    // float ms_off = 1;
    float ms_on = 20 - ms_off;
    outputPin.OutputPulsesStart(ms_on, ms_off, 0, true);
   
    // while (true) {
    //     // Sweep through all PWM duty cycle values from 0 to 255.
    //     for (int duty = 0; duty <= 255; duty++) {
    //      outputPin.PwmDuty(duty);
    //      Delay_ms(50);
    //     }

    //     // Sweep back down from 255 to 0.
    //     for (int duty = 255; duty >= 0; duty--) {
    //      outputPin.PwmDuty(duty);
    //      Delay_ms(50);
    //     }
    // }
 }