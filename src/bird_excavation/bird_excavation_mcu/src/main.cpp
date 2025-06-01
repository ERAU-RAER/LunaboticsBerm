#include <Arduino.h>
#include <HX711.h>

// Excavation System States
enum state { IDLE, ACTIVE };
enum state currentState;

enum command { CMD_STATE, CMD_POS, CMD_VIB, CMD_CAL, CMD_UNKNOWN = -1 };

// Timing
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL_MS = 1000;

// Status Code

enum StatusCode {
  STATUS_OK = 0,
  STATUS_ERR_POS = 1,
  STATUS_ERR_STATE = 2,
  STATUS_ERR_CAL = 3,
  STATUS_ERR_VIB = 4,
  STATUS_ERR_UNKNOWN = 99
};

StatusCode statusCode = STATUS_OK;

// LINACs
const uint8_t LINAC_00_EN = 2, LINAC_00_FWD = 3, LINAC_00_REV = 4;
const uint8_t LINAC_01_EN = 7, LINAC_01_FWD = 8, LINAC_01_REV = 9;
const uint8_t LINAC_02_EN = 25, LINAC_02_FWD = 27, LINAC_02_REV = 28;
const uint8_t LINAC_EN[3]  = { LINAC_00_EN, LINAC_01_EN, LINAC_02_EN };
const uint8_t LINAC_FWD[3] = { LINAC_00_FWD, LINAC_01_FWD, LINAC_02_FWD };
const uint8_t LINAC_REV[3] = { LINAC_00_REV, LINAC_01_REV, LINAC_02_REV };

// Hall Effect Sensors
const uint8_t HALL_00_A = 5, HALL_00_B = 6;
const uint8_t HALL_01_A = 10, HALL_01_B = 11;
const uint8_t HALL_02_A = 29, HALL_02_B = 30;

int16_t linAcPos[3] = { 0, 0, 0 };
int16_t linAcTargetPos[3] = { 0, 0, 0 };

// Vibration
const uint8_t VIB_EN = 22;

// Load Cells
HX711 LDCLL_00;
HX711 LDCLL_01;
HX711 LDCLL[2] = { LDCLL_00, LDCLL_01 };
const uint8_t LDCLL_SCK = 34;
const uint8_t LDCLL_DAT[2] = { 33, 37 };

// Prototypes
command hashCommand(String cmd);
void updateLinAcPos(uint8_t linAcID, char channel);
void hall00A_ISR();
void hall00B_ISR();
void hall01A_ISR();
void hall01B_ISR();
void hall02A_ISR();
void hall02B_ISR();
void updateLinAcPos(uint8_t linAcID, char channel);


void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(LINAC_EN[i], OUTPUT);
    pinMode(LINAC_FWD[i], OUTPUT);
    pinMode(LINAC_REV[i], OUTPUT);
  }

  pinMode(VIB_EN, OUTPUT);

  pinMode(HALL_00_A, INPUT); pinMode(HALL_00_B, INPUT);
  pinMode(HALL_01_A, INPUT); pinMode(HALL_01_B, INPUT);
  pinMode(HALL_02_A, INPUT); pinMode(HALL_02_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_00_A), hall00A_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_00_B), hall00B_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_01_A), hall01A_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_01_B), hall01B_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_02_A), hall02A_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_02_B), hall02B_ISR, RISING);

  currentState = IDLE;
  Serial.println("INITIALIZED");
}

void loop() {
  // Serial Command Handling
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\r\n');
    input.trim();
    if (input.length() == 0) return;

    int sepIndex = input.indexOf(':');
    String cmd = input;
    String args = "";

    if (sepIndex != -1) {
      cmd = input.substring(0, sepIndex);
      args = input.substring(sepIndex + 1);
    }

    cmd.trim(); args.trim();

    switch (hashCommand(cmd)) {
      case CMD_STATE:
        if (args == "IDLE") {
          currentState = IDLE;
          statusCode = STATUS_OK;
        } else if (args == "ACTIVE") {
          currentState = ACTIVE;
          statusCode = STATUS_OK;
        } else {
          statusCode = STATUS_ERR_STATE;
        }
        break;

      case CMD_POS:
        if (currentState == ACTIVE) {
          int c1 = args.indexOf(',');
          int c2 = args.indexOf(',', c1 + 1);
          if (c1 != -1 && c2 != -1) {
            linAcTargetPos[0] = args.substring(0, c1).toInt();
            linAcTargetPos[1] = args.substring(c1 + 1, c2).toInt();
            linAcTargetPos[2] = args.substring(c2 + 1).toInt();
            statusCode = STATUS_OK;
          } else {
            statusCode = STATUS_ERR_POS;
          }
        } else {
          statusCode = STATUS_ERR_STATE;
        }
        break;

      case CMD_CAL:
        if (currentState == ACTIVE) currentState = IDLE;  // Fix typo
        if (args == "ARM") {
          statusCode = STATUS_OK;
        } else if (args == "LDCL") {
          statusCode = STATUS_OK;
        } else {
          statusCode = STATUS_ERR_CAL;
        }
        break;

      case CMD_VIB:
        if (args == "ON") {
          digitalWrite(VIB_EN, HIGH);
          statusCode = STATUS_OK;
        } else if (args == "OFF") {
          digitalWrite(VIB_EN, LOW);
          statusCode = STATUS_OK;
        } else {
          statusCode = STATUS_ERR_VIB;
        }
        break;

      default:
        statusCode = STATUS_ERR_UNKNOWN;
        break;
    }
  }

  // === HEARTBEAT === //
  if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = millis();

    // Read load cells (stand in)
    long ldcl0 = 0;
    long ldcl1 = 0;

    Serial.print(currentState == IDLE ? "IDLE" : "ACTIVE"); Serial.print(',');
    Serial.print(linAcPos[0]); Serial.print(',');
    Serial.print(linAcPos[1]); Serial.print(',');
    Serial.print(linAcPos[2]); Serial.print(',');
    Serial.print(ldcl0); Serial.print(',');
    Serial.print(ldcl1); Serial.print(',');
    Serial.println(statusCode);
  }

  // === LINAC CONTROL === //
  if (currentState == ACTIVE) {
    for (uint8_t i = 0; i < 3; i++) {
      if (linAcPos[i] < linAcTargetPos[i]) {
        digitalWrite(LINAC_EN[i], HIGH);
        digitalWrite(LINAC_FWD[i], HIGH);
        digitalWrite(LINAC_REV[i], LOW);
      } else if (linAcPos[i] > linAcTargetPos[i]) {
        digitalWrite(LINAC_EN[i], HIGH);
        digitalWrite(LINAC_FWD[i], LOW);
        digitalWrite(LINAC_REV[i], HIGH);
      } else {
        digitalWrite(LINAC_EN[i], LOW);
        digitalWrite(LINAC_FWD[i], LOW);
        digitalWrite(LINAC_REV[i], LOW);
      }
    }
  } else {
    for (uint8_t i = 0; i < 3; i++) {
      digitalWrite(LINAC_EN[i], LOW);
      digitalWrite(LINAC_FWD[i], LOW);
      digitalWrite(LINAC_REV[i], LOW);
    }
  }
}

// === HELPER FUNCTIONS === //

command hashCommand(String cmd) {
  cmd.trim(); cmd.toUpperCase();
  if (cmd == "STATE") return CMD_STATE;
  if (cmd == "POS") return CMD_POS;
  if (cmd == "VIB") return CMD_VIB;
  if (cmd == "CAL") return CMD_CAL;
  return CMD_UNKNOWN;
}

void updateLinAcPos(uint8_t linAcID, char channel) {
  uint8_t hallA, hallB;
  switch (linAcID) {
    case 0: hallA = HALL_00_A; hallB = HALL_00_B; break;
    case 1: hallA = HALL_01_A; hallB = HALL_01_B; break;
    case 2: hallA = HALL_02_A; hallB = HALL_02_B; break;
    default: return;
  }

  if (channel == 'A' && digitalRead(hallB) == LOW) linAcPos[linAcID]++;
  else if (channel == 'B' && digitalRead(hallA) == LOW) linAcPos[linAcID]--;
}

// === ISRs === //
void hall00A_ISR() { updateLinAcPos(0, 'A'); }
void hall00B_ISR() { updateLinAcPos(0, 'B'); }
void hall01A_ISR() { updateLinAcPos(1, 'A'); }
void hall01B_ISR() { updateLinAcPos(1, 'B'); }
void hall02A_ISR() { updateLinAcPos(2, 'A'); }
void hall02B_ISR() { updateLinAcPos(2, 'B'); }