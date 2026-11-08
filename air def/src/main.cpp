#include <Arduino.h>
#include <ESP32Servo.h>
#include <cstdlib>
#include <cstring>

// HC-SR04 pins. ECHO must be reduced to 3.3 V with a voltage divider.
constexpr uint8_t TRIG_PIN = 25;
constexpr uint8_t ECHO_PIN = 26;
constexpr uint8_t RADAR_SERVO_PIN = 27;

// The SG90 is a 180-degree servo. Keep a margin away from its mechanical stops.
constexpr int SERVO_MIN_DEG = 10;
constexpr int SERVO_MAX_DEG = 170;
constexpr int SERVO_STEP_DEG = 2;
constexpr uint32_t ECHO_TIMEOUT_US = 30000UL;
constexpr uint32_t SERVO_SETTLE_MS = 55;
constexpr uint32_t PING_INTERVAL_MS = 35;
constexpr uint32_t MAX_SCAN_DURATION_MS = 3600000UL;
// Lock mode moves fast: shorter ping cadence and no settle wait so the servo
// can keep up with a re-lock stream arriving every ~80 ms from the backend.
constexpr uint32_t LOCK_PING_INTERVAL_MS = 25;

Servo radarServo;
int servoAngle = SERVO_MIN_DEG;
int sweepDirection = 1;
bool scanning = false;                // Wait for the dashboard to start a scan.
bool locked = false;                  // Point-and-follow mode driven by LOCK commands.
int lockAngle = 90;                   // Servo angle held while locked.
uint32_t scanStartedAt = 0;
uint32_t scanDurationMs = 0;           // 0 means continuous.
uint32_t servoMovedAt = 0;
uint32_t lastPingAt = 0;
uint32_t lastLockPingAt = 0;
char commandBuffer[48];
size_t commandLength = 0;

void setServoAngle(int angle) {
    servoAngle = constrain(angle, SERVO_MIN_DEG, SERVO_MAX_DEG);
    radarServo.write(servoAngle);
    servoMovedAt = millis();
}

void startScan(uint32_t durationMs) {
    locked = false;
    scanDurationMs = min(durationMs, MAX_SCAN_DURATION_MS);
    scanStartedAt = millis();
    scanning = true;
    sweepDirection = 1;
    setServoAngle(SERVO_MIN_DEG);
    Serial.println("#SCAN,STARTED");
}

void stopScan(const char *reason) {
    scanning = false;
    scanDurationMs = 0;
    Serial.print("#SCAN,STOPPED,");
    Serial.println(reason);
}

void handleCommand(char *line) {
    while (*line == ' ' || *line == '\t') ++line;

    if (strncmp(line, "UNLOCK", 6) == 0) {
        locked = false;
        setServoAngle(90);
        Serial.println("#LOCK,RELEASED");
        return;
    }

    if (strncmp(line, "LOCK", 4) == 0) {
        char *value = line + 4;
        while (*value == ' ' || *value == '\t') ++value;
        if (*value == '\0') {
            Serial.println("#ERROR,LOCK_NEEDS_ANGLE");
            return;
        }
        char *end = nullptr;
        long bearing = strtol(value, &end, 10);
        if (end == value || (*end != '\0' && *end != '\r')) {
            Serial.println("#ERROR,INVALID_BEARING");
            return;
        }
        int servoDeg = constrain(static_cast<int>(bearing) + 90,
                                 SERVO_MIN_DEG, SERVO_MAX_DEG);
        scanning = false;
        locked = true;
        lockAngle = servoDeg;
        setServoAngle(servoDeg);
        Serial.printf("#LOCK,STARTED,%ld\n", bearing);
        return;
    }

    if (strncmp(line, "STOP", 4) == 0) {
        stopScan("command");
        return;
    }

    if (strncmp(line, "START", 5) == 0) {
        char *value = line + 5;
        while (*value == ' ' || *value == '\t') ++value;
        uint32_t durationMs = 0;
        if (*value != '\0') {
            char *end = nullptr;
            unsigned long parsed = strtoul(value, &end, 10);
            if (end == value || (*end != '\0' && *end != '\r')) {
                Serial.println("#ERROR,INVALID_DURATION");
                return;
            }
            durationMs = static_cast<uint32_t>(parsed);
        }
        startScan(durationMs);
        return;
    }

    Serial.println("#ERROR,UNKNOWN_COMMAND");
}

void readSerialCommands() {
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\n' || ch == '\r') {
            if (commandLength > 0) {
                commandBuffer[commandLength] = '\0';
                handleCommand(commandBuffer);
                commandLength = 0;
            }
        } else if (commandLength < sizeof(commandBuffer) - 1) {
            commandBuffer[commandLength++] = ch;
        } else {
            commandLength = 0;
            Serial.println("#ERROR,COMMAND_TOO_LONG");
        }
    }
}

float readDistanceCm() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(3);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    const unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
    if (duration == 0) return -1.0f;

    const float distanceCm = duration * 0.0343f / 2.0f;
    if (distanceCm < 2.0f || distanceCm > 400.0f) return -1.0f;
    return distanceCm;
}

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    radarServo.setPeriodHertz(50);
    radarServo.attach(RADAR_SERVO_PIN, 500, 2400);

    Serial.println("#ESP32_RADAR_READY");
    Serial.println("#FORMAT,timestamp_ms,bearing_deg,distance_cm");
    setServoAngle(90);                 // Park at the centre while idle.
    Serial.println("#SCAN,STOPPED,boot");
}

void loop() {
    readSerialCommands();

    if (locked) {
        const uint32_t now = millis();
        if (static_cast<uint32_t>(now - lastLockPingAt) < LOCK_PING_INTERVAL_MS) return;
        lastLockPingAt = now;

        const float distanceCm = readDistanceCm();
        const int bearingDeg = lockAngle - 90;
        Serial.printf("%lu,%d,%.2f\n",
                      static_cast<unsigned long>(millis()), bearingDeg, distanceCm);
        return;
    }

    if (!scanning) return;

    const uint32_t now = millis();
    if (scanDurationMs > 0 && static_cast<uint32_t>(now - scanStartedAt) >= scanDurationMs) {
        stopScan("duration_elapsed");
        return;
    }

    if (static_cast<uint32_t>(now - servoMovedAt) < SERVO_SETTLE_MS) return;
    if (static_cast<uint32_t>(now - lastPingAt) < PING_INTERVAL_MS) return;
    lastPingAt = now;

    const float distanceCm = readDistanceCm();

    const int bearingDeg = servoAngle - 90;
    Serial.printf("%lu,%d,%.2f\n",
                  static_cast<unsigned long>(millis()), bearingDeg, distanceCm);

    int nextAngle = servoAngle + sweepDirection * SERVO_STEP_DEG;
    if (nextAngle >= SERVO_MAX_DEG) {
        nextAngle = SERVO_MAX_DEG;
        sweepDirection = -1;
    } else if (nextAngle <= SERVO_MIN_DEG) {
        nextAngle = SERVO_MIN_DEG;
        sweepDirection = 1;
    }
    setServoAngle(nextAngle);
}