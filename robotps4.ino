#include "Bluepad32.h"
#include <ESP32Servo.h>
#include <Preferences.h>

// Chassis motor pins
#define ENA 12
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25
#define ENB 33

// Arm servo pins
#define BASE_PIN 15
#define SHOULDER_PIN 2
#define ELBOW_PIN 4
#define GRIPPER_PIN 16

const int STICK_DEADZONE = 70;
const int ARM_STEP_DEGREES = 3;
const int R1_GRIPPER_OPEN_POS = 90;
const int R1_GRIPPER_CLOSED_POS = 150;
const unsigned long STICK_REPORT_INTERVAL_MS = 200;

ControllerPtr ps4Controller = nullptr;

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;

Preferences controllerPrefs;
uint8_t authorizedBtAddr[6] = {0};
bool authorizedAddressSaved = false;

int basePos = 90;
int shoulderPos = 90;
int elbowPos = 90;
int gripperPos = R1_GRIPPER_OPEN_POS;

bool triangleWasPressed = false;
bool crossWasPressed = false;
bool squareWasPressed = false;
bool circleWasPressed = false;
bool r1WasPressed = false;
bool gripperIsClosed = false;

unsigned long lastStickReport = 0;

void driveMotors(int leftSpeed, int rightSpeed);

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

    // Enroll the first controller if no PS4 controller is saved yet.
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

        Serial.printf(
            "Enrolled PS4 controller: %02X:%02X:%02X:%02X:%02X:%02X\n",
            authorizedBtAddr[0], authorizedBtAddr[1], authorizedBtAddr[2],
            authorizedBtAddr[3], authorizedBtAddr[4], authorizedBtAddr[5]
        );
    }

    if (!isAuthorizedController(ctl)) {
        Serial.println("Rejected: not the enrolled PS4 controller.");
        ctl->disconnect();
        return;
    }

    if (ps4Controller == nullptr) {
        ps4Controller = ctl;
        Serial.println("Authorized PS4 controller connected.");
    } else if (ps4Controller != ctl) {
        ctl->disconnect();
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (ps4Controller == ctl) {
        ps4Controller = nullptr;
        driveMotors(0, 0);

        triangleWasPressed = false;
        crossWasPressed = false;
        squareWasPressed = false;
        circleWasPressed = false;
        r1WasPressed = false;

        Serial.println("PS4 controller disconnected; motors stopped.");
    }
}

int applyDeadzone(int value) {
    return (abs(value) < STICK_DEADZONE) ? 0 : value;
}

void updateDrive() {
    int rawX = ps4Controller->axisX();
    int rawY = ps4Controller->axisY();

    // Negative stick Y means forward.
    int turn = applyDeadzone(rawX);
    int throttle = applyDeadzone(-rawY);

    int leftSpeed = map(throttle + turn, -1024, 1024, -255, 255);
    int rightSpeed = map(throttle - turn, -1024, 1024, -255, 255);

    driveMotors(leftSpeed, rightSpeed);

    // Report stick values and the interpreted action every 200 ms.
    if (millis() - lastStickReport >= STICK_REPORT_INTERVAL_MS) {
        lastStickReport = millis();

        const char* action;

        if (throttle > STICK_DEADZONE) {
            if (turn > STICK_DEADZONE) {
                action = "forward + right turn";
            } else if (turn < -STICK_DEADZONE) {
                action = "forward + left turn";
            } else {
                action = "forward";
            }
        } else if (throttle < -STICK_DEADZONE) {
            if (turn > STICK_DEADZONE) {
                action = "reverse + right turn";
            } else if (turn < -STICK_DEADZONE) {
                action = "reverse + left turn";
            } else {
                action = "reverse";
            }
        } else if (turn > STICK_DEADZONE) {
            action = "pivot right";
        } else if (turn < -STICK_DEADZONE) {
            action = "pivot left";
        } else {
            action = "stop / stick centered";
        }

        Serial.printf(
            "Left stick: X=%d Y=%d -> %s | motor command L=%d R=%d\n",
            rawX, rawY, action, leftSpeed, rightSpeed
        );
    }
}

void updateArm() {
    // Bluepad32 face-button mapping:
    // Triangle = y, Cross = a, Square = x, Circle = b.
    bool trianglePressed = ps4Controller->y();
    bool crossPressed = ps4Controller->a();
    bool squarePressed = ps4Controller->x();
    bool circlePressed = ps4Controller->b();

    // One 3-degree shoulder step per new button press.
    if (trianglePressed && !triangleWasPressed) {
        shoulderPos = constrain(
            shoulderPos + ARM_STEP_DEGREES, 0, 180
        );
        shoulderServo.write(shoulderPos);
        Serial.printf(
            "Triangle -> shoulder up; angle=%d\n",
            shoulderPos
        );
    }

    if (crossPressed && !crossWasPressed) {
        shoulderPos = constrain(
            shoulderPos - ARM_STEP_DEGREES, 0, 180
        );
        shoulderServo.write(shoulderPos);
        Serial.printf(
            "Cross -> shoulder down; angle=%d\n",
            shoulderPos
        );
    }

    // One 3-degree base step per new button press.
    if (squarePressed && !squareWasPressed) {
        basePos = constrain(basePos + ARM_STEP_DEGREES, 0, 180);
        baseServo.write(basePos);
        Serial.printf(
            "Square -> base left; angle=%d\n",
            basePos
        );
    }

    if (circlePressed && !circleWasPressed) {
        basePos = constrain(basePos - ARM_STEP_DEGREES, 0, 180);
        baseServo.write(basePos);
        Serial.printf(
            "Circle -> base right; angle=%d\n",
            basePos
        );
    }

    // R1 toggles between pick (close) and drop (open).
    bool r1Pressed = ps4Controller->r1();

    if (r1Pressed && !r1WasPressed) {
        gripperIsClosed = !gripperIsClosed;
        gripperPos = gripperIsClosed
            ? R1_GRIPPER_CLOSED_POS
            : R1_GRIPPER_OPEN_POS;

        gripperServo.write(gripperPos);

        Serial.printf(
            "R1 -> %s gripper; angle=%d\n",
            gripperIsClosed ? "PICK (close)" : "DROP (open)",
            gripperPos
        );
    }

    // Save current button states to prevent repeat movement while held.
    triangleWasPressed = trianglePressed;
    crossWasPressed = crossPressed;
    squareWasPressed = squarePressed;
    circleWasPressed = circlePressed;
    r1WasPressed = r1Pressed;
}

void driveMotors(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Left motor direction and speed
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

    // Right motor direction and speed
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

void setup() {
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    driveMotors(0, 0);

    baseServo.attach(BASE_PIN);
    shoulderServo.attach(SHOULDER_PIN);
    elbowServo.attach(ELBOW_PIN);
    gripperServo.attach(GRIPPER_PIN);

    baseServo.write(basePos);
    shoulderServo.write(shoulderPos);
    elbowServo.write(elbowPos);
    gripperServo.write(gripperPos);

    // Separate saved controller identity for the PS4 sketch.
    controllerPrefs.begin("robotps4", false);

    authorizedAddressSaved =
        controllerPrefs.getBytesLength("controller") == sizeof(authorizedBtAddr);

    if (authorizedAddressSaved) {
        controllerPrefs.getBytes(
            "controller",
            authorizedBtAddr,
            sizeof(authorizedBtAddr)
        );
    }

    // Touchpad mouse is not needed for robot control.
    BP32.enableVirtualDevice(false);
    BP32.setup(&onConnectedController, &onDisconnectedController);

    Serial.println("Ready. Connect the enrolled PS4 controller.");
}

void loop() {
    BP32.update();

    if (ps4Controller && ps4Controller->isConnected()) {
        updateDrive();
        updateArm();
    } else {
        driveMotors(0, 0);
    }
}