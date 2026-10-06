#include <Bluepad32.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>

// ============================================================
// MOTOR DRIVER PINS
// Retained from the earlier motor pinout. Verify against wiring.
// ============================================================
#define ENA 12
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25
#define ENB 33

// ============================================================
// I2C / PCA9685
// SDA=21 and SCL=22 are from the handwritten pinout.
// ============================================================
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define PCA9685_ADDRESS 0x40  // TODO: confirm board address

Adafruit_PWMServoDriver pca(PCA9685_ADDRESS);
bool pcaReady = false;

// TODO: confirm servo channel wiring on the PCA9685.
const uint8_t BASE_SERVO_CHANNEL = 0;
const uint8_t SHOULDER_SERVO_CHANNEL = 1;
const uint8_t ELBOW_SERVO_CHANNEL = 2;
const uint8_t GRIPPER_SERVO_CHANNEL = 3;

// TODO: calibrate these pulse limits for your servos and linkage.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_FREQUENCY_HZ = 50;

const int ARM_STEP_DEGREES = 3;
const int GRIPPER_OPEN_ANGLE = 90;    // TODO: calibrate
const int GRIPPER_CLOSED_ANGLE = 150; // TODO: calibrate

// ============================================================
// CONTROLLER / STATE
// ============================================================
ControllerPtr xboxController = nullptr;

Preferences controllerPrefs;
uint8_t authorizedBtAddr[6] = {0};
bool authorizedAddressSaved = false;

int basePos = 90;
int shoulderPos = 90;
int elbowPos = 90;
int gripperPos = GRIPPER_OPEN_ANGLE;
bool gripperIsClosed = false;

// D-pad is the default drive control. Menu toggles to the right stick.
bool useRightStickForDrive = false;

bool menuWasPressed = false;
bool yWasPressed = false;
bool aWasPressed = false;
bool xWasPressed = false;
bool bWasPressed = false;
bool lbWasPressed = false;

const int RIGHT_STICK_DEADZONE = 70;
const unsigned long REPORT_INTERVAL_MS = 200;
unsigned long lastReportMs = 0;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed);
void stopMotors();

// ============================================================
// CONTROLLER ID LOCK
// The first Xbox controller to connect is saved in Preferences.
// This namespace is separate from the PS4 sketch's namespace.
// ============================================================
bool isAuthorizedController(ControllerPtr ctl) {
    ControllerProperties properties = ctl->getProperties();

    for (int i = 0; i < 6; ++i) {
        if (properties.btaddr[i] != authorizedBtAddr[i]) {
            return false;
        }
    }
    return true;
}

void onConnectedController(ControllerPtr ctl) {
    ControllerProperties properties = ctl->getProperties();

    if (!authorizedAddressSaved) {
        for (int i = 0; i < 6; ++i) {
            authorizedBtAddr[i] = properties.btaddr[i];
        }

        controllerPrefs.putBytes(
            "controller",
            authorizedBtAddr,
            sizeof(authorizedBtAddr)
        );
        authorizedAddressSaved = true;
        Serial.println("First connected controller enrolled for Xbox sketch.");
    }

    if (!isAuthorizedController(ctl)) {
        Serial.println("Rejected: this is not the enrolled Xbox controller.");
        ctl->disconnect();
        return;
    }

    if (xboxController == nullptr) {
        xboxController = ctl;
        Serial.println("Authorized Xbox controller connected.");
    } else if (xboxController != ctl) {
        ctl->disconnect();
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (xboxController == ctl) {
        xboxController = nullptr;
        stopMotors();

        menuWasPressed = false;
        yWasPressed = false;
        aWasPressed = false;
        xWasPressed = false;
        bWasPressed = false;
        lbWasPressed = false;

        Serial.println("Xbox controller disconnected; motors stopped.");
    }
}

// ============================================================
// SERVO OUTPUT THROUGH PCA9685
// ============================================================
void writeServoAngle(uint8_t channel, int angle) {
    angle = constrain(angle, 0, 180);

    if (!pcaReady) {
        Serial.println("PCA9685 unavailable; servo command not sent.");
        return;
    }

    int pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
    int ticks = (pulseUs * 4096L * SERVO_FREQUENCY_HZ) / 1000000L;
    ticks = constrain(ticks, 0, 4095);

    pca.setPWM(channel, 0, ticks);
}

// ============================================================
// DRIVE CONTROL
// ============================================================
int applyDeadzone(int value) {
    return (abs(value) < RIGHT_STICK_DEADZONE) ? 0 : value;
}

void updateDrive() {
    int leftSpeed = 0;
    int rightSpeed = 0;
    const char* action = "stop";

    if (useRightStickForDrive) {
        // Right stick: horizontal turns, vertical drives.
        int rawX = xboxController->axisRX();
        int rawY = xboxController->axisRY();

        int turn = applyDeadzone(rawX);
        int throttle = applyDeadzone(-rawY);

        leftSpeed = map(throttle + turn, -1024, 1024, -255, 255);
        rightSpeed = map(throttle - turn, -1024, 1024, -255, 255);

        if (throttle > RIGHT_STICK_DEADZONE) {
            if (turn > RIGHT_STICK_DEADZONE) action = "forward + right turn";
            else if (turn < -RIGHT_STICK_DEADZONE) action = "forward + left turn";
            else action = "forward";
        } else if (throttle < -RIGHT_STICK_DEADZONE) {
            if (turn > RIGHT_STICK_DEADZONE) action = "reverse + right turn";
            else if (turn < -RIGHT_STICK_DEADZONE) action = "reverse + left turn";
            else action = "reverse";
        } else if (turn > RIGHT_STICK_DEADZONE) {
            action = "pivot right";
        } else if (turn < -RIGHT_STICK_DEADZONE) {
            action = "pivot left";
        }

        if (millis() - lastReportMs >= REPORT_INTERVAL_MS) {
            lastReportMs = millis();
            Serial.printf(
                "Drive=RIGHT STICK | RX=%d RY=%d -> %s | L=%d R=%d\n",
                rawX, rawY, action, leftSpeed, rightSpeed
            );
        }
    } else {
        // D-pad tank-style drive.
        uint8_t dpad = xboxController->dpad();

        if (dpad & DPAD_UP) {
            leftSpeed = 200;
            rightSpeed = 200;
            action = "forward";
        } else if (dpad & DPAD_DOWN) {
            leftSpeed = -200;
            rightSpeed = -200;
            action = "reverse";
        } else if (dpad & DPAD_LEFT) {
            leftSpeed = -180;
            rightSpeed = 180;
            action = "pivot left";
        } else if (dpad & DPAD_RIGHT) {
            leftSpeed = 180;
            rightSpeed = -180;
            action = "pivot right";
        }

        if (millis() - lastReportMs >= REPORT_INTERVAL_MS) {
            lastReportMs = millis();
            Serial.printf(
                "Drive=D-PAD | value=0x%02X -> %s | L=%d R=%d\n",
                dpad, action, leftSpeed, rightSpeed
            );
        }
    }

    driveMotors(leftSpeed, rightSpeed);
}

// ============================================================
// ARM AND GRIPPER BUTTONS
// Bluepad32 positional mapping:
// Y=top, A=bottom, X=left, B=right.
// ============================================================
void updateArm() {
    bool yPressed = xboxController->y();
    bool aPressed = xboxController->a();
    bool xPressed = xboxController->x();
    bool bPressed = xboxController->b();

    // One shoulder step per new press.
    if (yPressed && !yWasPressed) {
        shoulderPos = constrain(shoulderPos + ARM_STEP_DEGREES, 0, 180);
        writeServoAngle(SHOULDER_SERVO_CHANNEL, shoulderPos);
        Serial.printf("Y -> shoulder up; angle=%d\n", shoulderPos);
    }

    if (aPressed && !aWasPressed) {
        shoulderPos = constrain(shoulderPos - ARM_STEP_DEGREES, 0, 180);
        writeServoAngle(SHOULDER_SERVO_CHANNEL, shoulderPos);
        Serial.printf("A -> shoulder down; angle=%d\n", shoulderPos);
    }

    // One base step per new press.
    if (xPressed && !xWasPressed) {
        basePos = constrain(basePos + ARM_STEP_DEGREES, 0, 180);
        writeServoAngle(BASE_SERVO_CHANNEL, basePos);
        Serial.printf("X -> base left; angle=%d\n", basePos);
    }

    if (bPressed && !bWasPressed) {
        basePos = constrain(basePos - ARM_STEP_DEGREES, 0, 180);
        writeServoAngle(BASE_SERVO_CHANNEL, basePos);
        Serial.printf("B -> base right; angle=%d\n", basePos);
    }

    // LB toggles between pick/close and drop/open.
    bool lbPressed = xboxController->l1();

    if (lbPressed && !lbWasPressed) {
        gripperIsClosed = !gripperIsClosed;
        gripperPos = gripperIsClosed
            ? GRIPPER_CLOSED_ANGLE
            : GRIPPER_OPEN_ANGLE;

        writeServoAngle(GRIPPER_SERVO_CHANNEL, gripperPos);

        Serial.printf(
            "LB -> %s gripper; angle=%d\n",
            gripperIsClosed ? "PICK (close)" : "DROP (open)",
            gripperPos
        );
    }

    yWasPressed = yPressed;
    aWasPressed = aPressed;
    xWasPressed = xPressed;
    bWasPressed = bPressed;
    lbWasPressed = lbPressed;
}

// ============================================================
// MOTOR OUTPUT
// Uses the earlier supplied L298N/TB6612-style pin arrangement.
// Verify the actual driver and wiring before powering motors.
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else if (leftSpeed < 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }
    analogWrite(ENA, abs(leftSpeed));

    if (rightSpeed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else if (rightSpeed < 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    }
    analogWrite(ENB, abs(rightSpeed));
}

void stopMotors() {
    driveMotors(0, 0);
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    stopMotors();

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    pcaReady = pca.begin();
    if (pcaReady) {
        pca.setPWMFreq(SERVO_FREQUENCY_HZ);

        writeServoAngle(BASE_SERVO_CHANNEL, basePos);
        writeServoAngle(SHOULDER_SERVO_CHANNEL, shoulderPos);
        writeServoAngle(ELBOW_SERVO_CHANNEL, elbowPos);
        writeServoAngle(GRIPPER_SERVO_CHANNEL, gripperPos);

        Serial.println("PCA9685 ready; servo frequency set to 50 Hz.");
    } else {
        Serial.println("PCA9685 not found; controller diagnostics will still run.");
    }

    controllerPrefs.begin("robotxbox", false);
    authorizedAddressSaved =
        controllerPrefs.getBytesLength("controller") == sizeof(authorizedBtAddr);

    if (authorizedAddressSaved) {
        controllerPrefs.getBytes(
            "controller",
            authorizedBtAddr,
            sizeof(authorizedBtAddr)
        );
    }

    BP32.setup(&onConnectedController, &onDisconnectedController);
    Serial.println("Ready. D-pad drives by default; Menu toggles D-pad/right-stick.");
}

void loop() {
    BP32.update();

    if (!xboxController || !xboxController->isConnected()) {
        stopMotors();
        delay(10);
        return;
    }

    bool menuPressed =
        (xboxController->miscButtons() & MISC_BUTTON_START) != 0;

    if (menuPressed && !menuWasPressed) {
        useRightStickForDrive = !useRightStickForDrive;
        stopMotors();

        Serial.println(
            useRightStickForDrive
                ? "Menu -> drive control switched to RIGHT STICK"
                : "Menu -> drive control switched to D-PAD"
        );
    }
    menuWasPressed = menuPressed;

    updateDrive();
    updateArm();
}