// Standalone PID bench test with AutoTune
// Type 'tune' to auto-tune, then type 0-100 to test positions.

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <QuickPID.h>
#include <sTune.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can0;
IntervalTimer canTimer;

// --- State ---
volatile uint8_t reportedPosition = 0;
volatile uint16_t rawPosition = 0;
float pidInput = 50.0f;
float pidOutput = 50.0f;
float pidSetpoint = 50.0f;

float Kp = 1.0f, Ki = 0.2f, Kd = 0.0f;

QuickPID pid(&pidInput, &pidOutput, &pidSetpoint,
    Kp, Ki, Kd,
    QuickPID::pMode::pOnError,
    QuickPID::dMode::dOnMeas,
    QuickPID::iAwMode::iAwClamp,
    QuickPID::Action::direct);

// --- AutoTune ---
sTune tuner(&pidInput, &pidOutput, tuner.ZN_PID, tuner.directIP, tuner.printALL);
bool tuning = false;
bool tuned = false;

// --- CAN ---
void canSniff(const CAN_message_t &msg) {
    if (msg.id != 0x4EB) return;
    rawPosition = (msg.buf[2] << 8) | msg.buf[3];
    reportedPosition = 100 - map(rawPosition, 0, 1000, 0, 100);
}

void sendCanPosition(uint8_t position) {
    CAN_message_t msg;
    msg.id = 0x4EA;
    msg.flags.extended = 0;
    msg.len = 8;
    memset(msg.buf, 0, 8);
    msg.buf[0] = map(100 - position, 0, 100, 0, 250);
    Can0.write(msg);
}

void canTimerISR() {
    if (tuning) {
        sendCanPosition((uint8_t)(pidOutput + 0.5f));
    } else {
        sendCanPosition((uint8_t)(pidOutput + 0.5f));
    }
}

// --- Serial ---
void handleSerial() {
    static char buf[32];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (idx == 0) continue;
            buf[idx] = '\0';

            if (strcmp(buf, "tune") == 0) {
                if (tuning) {
                    Serial.println("Already tuning...");
                } else {
                    tuning = true;
                    pid.SetMode(QuickPID::Control::manual);
                    // Configure: inputSpan, outputSpan, outputStart, outputStep,
                    //            testTimeSec, settleTimeSec, samples
                    tuner.Configure(100.0f, 100.0f, pidOutput, pidOutput + 10.0f,
                                    10.0f, 2.0f, 100);
                    tuner.SetEmergencyStop(100.0f);
                    Serial.println("AutoTune started — sit tight...");
                }
            } else if (strncmp(buf, "pid ", 4) == 0) {
                float kp, ki, kd;
                if (sscanf(buf + 4, "%f %f %f", &kp, &ki, &kd) == 3) {
                    Kp = kp; Ki = ki; Kd = kd;
                    pid.SetTunings(Kp, Ki, Kd);
                    Serial.println("PID tunings updated");
                } else {
                    Serial.println("Usage: pid <Kp> <Ki> <Kd>");
                }
            } else if (strcmp(buf, "pid") == 0) {
                char tbuf[64];
                snprintf(tbuf, sizeof(tbuf), "Kp=%.2f Ki=%.2f Kd=%.2f",
                    (double)Kp, (double)Ki, (double)Kd);
                Serial.println(tbuf);
            } else if (!tuning) {
                int val = atoi(buf);
                if (val >= 0 && val <= 100) {
                    pidSetpoint = (float)val;
                    Serial.print("Target: ");
                    Serial.print(val);
                    Serial.println("%");
                } else {
                    Serial.println("0-100, 'tune', 'pid', or 'pid <Kp> <Ki> <Kd>'");
                }
            }
            idx = 0;
        } else if (idx < 31) {
            buf[idx++] = c;
        }
    }
}

// --- Debug ---
IntervalTimer debugTimer;
volatile bool debugFlag = false;
void debugISR() { debugFlag = true; }

void printDebug() {
    if (!debugFlag) return;
    debugFlag = false;

    char buf[80];
    if (tuning) {
        snprintf(buf, sizeof(buf), "TUNING  Pos:%d%% Out:%.1f Raw:%u",
            reportedPosition, (double)pidOutput, rawPosition);
    } else {
        snprintf(buf, sizeof(buf), "Target:%d%% Pos:%d%% Out:%.1f Raw:%u",
            (int)pidSetpoint, reportedPosition, (double)pidOutput, rawPosition);
    }
    Serial.println(buf);
}

// --- Main ---
void setup() {
    Serial.begin(115200);

    Can0.begin();
    Can0.setBaudRate(500000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(canSniff);

    pid.SetOutputLimits(0.0f, 100.0f);
    pid.SetSampleTimeUs(100000);  // 100ms
    pid.SetMode(QuickPID::Control::automatic);

    canTimer.begin(canTimerISR, 20000);
    debugTimer.begin(debugISR, 1000000);

    Serial.println("PID Bench Test + AutoTune");
    Serial.println("Type 'tune' to auto-tune, 0-100 to set target, 'pid' to see tunings");
    Serial.println("Starting at 50%");
}

void loop() {
    handleSerial();

    static elapsedMillis loopTimer;
    if (loopTimer < 10) return;
    loopTimer = 0;

    pidInput = (float)reportedPosition;

    if (tuning) {
        int status = tuner.Run();
        if (status == tuner.tunings) {
            tuning = false;
            tuned = true;
            tuner.GetAutoTunings(&Kp, &Ki, &Kd);
            pid.SetTunings(Kp, Ki, Kd);
            pid.SetMode(QuickPID::Control::automatic);

            char tbuf[80];
            snprintf(tbuf, sizeof(tbuf), "AutoTune done! Kp=%.2f Ki=%.2f Kd=%.2f",
                (double)Kp, (double)Ki, (double)Kd);
            Serial.println(tbuf);
            Serial.println("Type 0-100 to test positions");
        }
    } else {
        pid.Compute();
    }

    printDebug();
}
