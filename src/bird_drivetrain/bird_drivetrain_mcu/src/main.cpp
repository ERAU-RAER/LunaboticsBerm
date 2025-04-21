#include <Arduino.h>
#include "ClearCore.h"
#include "mcvc_cmds.h"

#define baudRate 115200
#define SerialPort ConnectorUsb

MotorDriver* motors[] = { &ConnectorM0, &ConnectorM1, &ConnectorM2, &ConnectorM3 };

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

  // Enables the motor
  motors[0] -> EnableRequest(true);
  motors[1] -> EnableRequest(true);
  motors[2] -> EnableRequest(true);
  motors[3] -> EnableRequest(true);
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

  const int IN_BUFFER_LEN = 32;
  char input[IN_BUFFER_LEN+1];

  while (true) {

    // clear buffer
    for (int i = 0; i <= IN_BUFFER_LEN; ++i) {
      input[i] = '\0';
    }
    // read up to IN_BUFFER_LEN chars or until no more data
    bool overflow = false;
    int i = 0;
    while (i < IN_BUFFER_LEN && SerialPort.CharPeek() != -1) {
      input[i++] = (char)SerialPort.CharGet();
      Delay_ms(1);
    }
    // if there's still data, we overflowed
    if (SerialPort.CharPeek() != -1) {
      overflow = true;
      SerialPort.FlushInput();
    }
    // nothing read → skip
    if (i == 0) {
      continue;
    }
    if (overflow) {
      SerialPort.SendLine("ERR: buffer overrun");
      continue;
    }

    // command sanity: first must be letter
    char cmd = input[0];
    if (!((cmd >= 'a' && cmd <= 'z') || (cmd >= 'A' && cmd <= 'Z'))) {
      SerialPort.SendLine("ERR: invalid command");
      continue;
    }
    // normalize to lowercase
    if (cmd >= 'A' && cmd <= 'Z') {
      cmd = cmd - 'A' + 'a';
    }

    // parse motor number & argument
    int motorNum = atoi(&input[1]);               // digits after letter
    int32_t arg     = atoi(&input[2]);             // e.g. velocity
    if (motorNum < 0 || motorNum > 3) {
      SerialPort.SendLine("ERR: bad motor #");
      continue;
    }

    // punch it
    switch (cmd) {
      case 'v':  // velocity command: "v# <vel>"
        // safety: set torque limit before commanding
        CommandVelocity(arg, *motors[motorNum]);
        SerialPort.SendLine("OK: velocity set");
        break;

      // you can add other cases here (e, d, m, etc.)

      default:
        SerialPort.SendLine("ERR: unknown cmd");
        break;
    }
  }
}