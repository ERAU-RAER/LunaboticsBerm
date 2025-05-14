#include "drive.h"
#include <Arduino.h>

// stuff from main.cpp
extern MotorDriver* motors[];
extern bool           motorDirection[];

static double WHEEL_DIAM_M = 0.25192;  // meters
static double MAX_RPM      = 3000.0;   // RPM max

void initSpeedHLFB() {
    //Put HLFB on each motor into 0–100% speed PWM mode
    for (int i = 0; i < 4; ++i) {
        motors[i]->HlfbMode(MotorDriver::HLFB_MODE_HAS_PWM);
    }
}

void pushWheelSpeedsToJetson() {
    char buf[64];
    int  len = 0;

    for (int w = 0; w < 4; ++w) {
        auto& M = *motors[w];
        double speed_mps = 0.0;

        if (M.HlfbState() == MotorDriver::HLFB_HAS_MEASUREMENT) {
            double pct = M.HlfbPercent();   // 0 to 100
            //Convert to rev/s, then to m/s:
            double rev_per_s = (pct / 100.0) * (MAX_RPM / 60.0);
            speed_mps = rev_per_s * (M_PI * WHEEL_DIAM_M);
            //direction inversion stuff if needed
            if (!motorDirection[w]) speed_mps = -speed_mps;
        }

        // Append "M<w>:<speed>" plus comma]
        len += snprintf(buf + len, sizeof(buf) - len,
                             "M%d:%.3f%s",
                             w, speed_mps, (w < 3 ? "," : ""));
    }

    SerialPort.SendLine(buf);
}
