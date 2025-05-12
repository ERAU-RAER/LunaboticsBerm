#include <Arduino.h>
#include <ctype.h>
#include "ClearCore.h"
#include "mcvc_cmds.h"
#include "robot_characteristics.h"

#define baudRate 115200
#define SerialPort ConnectorUsb

MotorDriver* motors[] = { &ConnectorM0, &ConnectorM1, &ConnectorM2, &ConnectorM3 };
bool motorDirection[] = {1, 1, 0, 0}; // 1 : non-inverted - 0 : inverted
 
void handleCommand(const char* input) {
  // sanity: first char is command letter
  char cmd = input[0];
  if (cmd >= 'A' && cmd <= 'Z') cmd = cmd - 'A' + 'a';
  if (cmd < 'a' || cmd > 'z') {
      SerialPort.SendLine("ERR: invalid command");
      return;
  }

  // parse motor number (can be multi‐digit)
  int pos = 1, motorNum = 0;
  if (!isdigit(input[pos])) {
      SerialPort.SendLine("ERR: bad motor #");
      return;
  }
  while (isdigit(input[pos])) {
      motorNum = motorNum * 10 + (input[pos++] - '0');
  }
  if (motorNum < 0 || motorNum > 3) {
      SerialPort.SendLine("ERR: bad motor #");
      return;
  }

  // skip spaces, then parse argument
  while (input[pos] == ' ') pos++;
  int32_t arg = atoi(&input[pos]);

  switch (cmd) {
    case 'v':
      if (!motorDirection[motorNum]) {
        arg = -arg;
      }
      CommandVelocity(arg, *motors[motorNum]);
      
      {
        char msg[48];
        snprintf(msg, sizeof(msg), "OK: velocity set to %ld on v%d", (long)arg, motorNum);
        SerialPort.SendLine(msg);
      }
      
      break;
    default:
      SerialPort.SendLine("ERR: unknown cmd");
      break;
  }
}

// void getWheelOdom(){
//   double currentSpeed_rpm;
//   double currentSpeed_mps;
//   for (int i = 0; i < 4; i++) {
//       currentSpeed_rpm = GetSpeed(*motors[i]);
//       currentSpeed_mps = currentSpeed_rpm * wheel_diameter_ * M_PI;
      
//       char msg[64];
//       snprintf(msg, sizeof(msg), "v%d speed: %.2f", i, currentSpeed_mps);
//       SerialPort.SendLine(msg);
//   }
// }

int main() {
  // Sets all motor connectors to the correct mode for Follow Digital
  // Velocity, Bipolar PWM mode.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,Connector::CPM_MODE_A_PWM_B_PWM);

  SerialPort.Mode(Connector::USB_CDC);
  SerialPort.Speed(baudRate);
  uint32_t timeout = 5000;
  uint32_t startTime = Milliseconds();
  SerialPort.PortOpen();
  while (!SerialPort && Milliseconds() - startTime < timeout) {
      continue;
  }

  // Welcome message
  SerialPort.SendLine("Good Morning, B.I.R.D. ...");
  SerialPort.SendLine("╰╮╰╮╰╮");
  SerialPort.SendLine("╭━━━━━━━╮╱ ");
  SerialPort.SendLine("╰━━━━━━━╯╱ ");
  SerialPort.SendLine("┃╭╭╮┏┏┏┏┣━╮");
  SerialPort.SendLine("┃┃┃┃┣┣┣┣┃╱┃ ");
  SerialPort.SendLine("┃╰╰╯┃┃┗┗┣━╯ ");
  SerialPort.SendLine("╰━━━━━━━╯ ");

  // Enables the motor
  motors[0] -> EnableRequest(true);
  motors[1] -> EnableRequest(true);
  motors[2] -> EnableRequest(true);
  motors[3] -> EnableRequest(true);

  for (auto m : motors) {
    m->MotorInADuty(0.0);   // 0% → use primary (100%) torque limits
  }

  LimitTorque(100, *motors[0]);
  LimitTorque(100, *motors[1]);
  LimitTorque(100, *motors[2]);
  LimitTorque(100, *motors[3]);

  SerialPort.SendLine("Motors Enabled");

  // Waits for 5 seconds to allow the motor to come up to speed
  SerialPort.SendLine("Waiting for motor to reach speed...");
  startTime = Milliseconds();
  while (Milliseconds() - startTime < timeout) {
      continue;
  }
  SerialPort.SendLine("Motors Ready");

  const int IN_BUFFER_LEN = 64;
  char  input[IN_BUFFER_LEN+1];
  int   idx      = 0;
  bool  overflow = false;

  while (true) {
    // consume all available chars
    while (SerialPort.CharPeek() != -1) {
      char c = (char)SerialPort.CharGet();
      Delay_ms(1);
      if (c == '\r') {
        continue;
      } else if (c == '\n') {
        input[idx] = '\0';
        if (overflow) {
          SerialPort.SendLine("ERR: buffer overrun");
        } else if (idx > 0) {
          handleCommand(input);
        }
        idx      = 0;
        overflow = false;
      } else {
        if (idx < IN_BUFFER_LEN) {
          input[idx++] = c;
        } else {
          overflow = true;
        }
      }
    }
    // getWheelOdom();
  }
}