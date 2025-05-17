#include <Arduino.h>
#include <ctype.h>
#include "ClearCore.h"
#include "robot_characteristics.h"

#define baudRate 2000000
#define SerialPort ConnectorUsb
#define INPUT_A_FILTER 20

const int maxSpeed = 1000;

MotorDriver* motors[] = { &ConnectorM0, &ConnectorM1, &ConnectorM2, &ConnectorM3 };
bool motorDirection[] = {1, 1, 0, 0};
 
bool CommandVelocity(int32_t commandedVelocity, MotorDriver motor, bool direction) {
  if (abs(commandedVelocity) >= abs(maxSpeed)) {
      SerialPort.SendLine("Move rejected, requested velocity at or over the limit.");
      return false;
  }

  // Check if an alert is currently preventing motion
  if (motor.StatusReg().bit.AlertsPresent) {
      SerialPort.SendLine("Motor status: 'In Alert'. Move Canceled.");
      return false;
  }

  SerialPort.Send("Commanding velocity: ");
  SerialPort.SendLine(commandedVelocity);

  // Change ClearPath's Input A state to change direction.
  // Note: this section of code was included so this commandVelocity function 
  // could be used to command negative (opposite direction) velocity. However the 
  // analog signal used by this example only commands positive velocities.
  bool state = (commandedVelocity >= 0) ? false : true;
  if (direction) {
      state = !state;
  }
  motor.MotorInAState(state);

  // Delays to send the correct filtered direction.
  Delay_ms(20 + INPUT_A_FILTER);

  // Find the scaling factor of our velocity range mapped to the PWM duty
  // cycle range (255 is the max duty cycle).
  double scaleFactor = 255.0 / maxSpeed;

  // Scale the velocity command to our duty cycle range.
  uint8_t dutyRequest = abs(commandedVelocity) * scaleFactor;

  // Command the move.
  motor.MotorInBDuty(dutyRequest);

  return true;
}

void handleCommand(const char* input) {
    // Get the command letter and convert to lowercase
    char cmd = input[0];
    if (cmd >= 'A' && cmd <= 'Z') cmd = cmd - 'A' + 'a';
    if (cmd < 'a' || cmd > 'z') {
        SerialPort.SendLine("ERR: invalid command");
        return;
    }
    
    // For the velocity command, we now expect two floating point numbers:
    if (cmd == 'v') {
      int i_linear = 0, i_angular = 0;
      if (sscanf(input + 1, "%d %d", &i_linear, &i_angular) != 2) {
          SerialPort.SendLine("ERR: bad parameters");
          return;
      }
      // Convert back to original float values
      float linear = i_linear / 1000.0f;
      float angular = i_angular / 1000.0f;
      
      double wheel_circumference = M_PI * wheel_diameter_;
      double rpm_left  = ((linear - (angular * track_width_ / 2.0f)) * gear_ratio_ * 60.0) / wheel_circumference;
      double rpm_right = ((linear + (angular * track_width_ / 2.0f)) * gear_ratio_ * 60.0) / wheel_circumference;
      int vel_left  = std::lround(rpm_left);
      int vel_right = std::lround(rpm_right);
      
      CommandVelocity(vel_right,  *motors[0], motorDirection[0]);
      CommandVelocity(vel_right, *motors[1], motorDirection[1]);
      CommandVelocity(vel_left,  *motors[2], motorDirection[2]);
      CommandVelocity(vel_left, *motors[3], motorDirection[3]);
      return;
  }
    
    // For other commands, parse a motor number
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
    
    // Skip spaces, then parse argument
    while (input[pos] == ' ') pos++;
    int32_t arg = atoi(&input[pos]);

    switch (cmd) {
        case 'd':
            // Disable the selected motor
            motors[motorNum]->EnableRequest(false);
            SerialPort.Send("Motor ");
            SerialPort.SendLine(motorNum);
            SerialPort.SendLine(" disabled.");
            break;
        case 'e':
            // Enable the selected motor
            motors[motorNum]->EnableRequest(true);
            SerialPort.Send("Motor ");
            SerialPort.SendLine(motorNum);
            SerialPort.SendLine(" enabled.");
            break;
        default:
            SerialPort.SendLine("ERR: unknown cmd");
            break;
    }
}

void safeShutdown() {
  SerialPort.SendLine("Serial connection dropped. Initiating safe shutdown.");
  // Command zero velocity for all motors
  for (int i = 0; i < 4; i++) {
       CommandVelocity(0, *motors[i], motorDirection[i]);
  }
  // Wait for 3000ms before disabling motors
  Delay_ms(3000);
  // Disable all motors
  for (int i = 0; i < 4; i++) {
       motors[i]->EnableRequest(false);
  }
  SerialPort.SendLine("Motors disabled.");
}

int main() {

  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,Connector::CPM_MODE_A_DIRECT_B_PWM);

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

  for (auto m : motors) {
    m -> EnableRequest(true);
  }

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

        for (int i = 0; i < 4; i++) {
        if (!motors[i]->StatusReg().bit.Enabled) {
            SerialPort.Send("Unexpected disable on motor ");
            SerialPort.Send(i);
            SerialPort.SendLine(" detected. Resetting...");
            // Reset the motor by disabling then re-enabling.
            motors[i]->EnableRequest(false);
            Delay_ms(100);
            motors[i]->EnableRequest(true);
            SerialPort.Send("Motor ");
            SerialPort.Send(i);
            SerialPort.SendLine(" re-enabled.");
        }
    }

    if (!SerialPort) {
      safeShutdown();
    }
  }
}
