#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_VL53L0X.h>

// ============================================================
// PLACEHOLDER CONFIGURATION
// Fill these in from the actual maze and wiring before enabling.
// ============================================================

// Keep false until every hardware hook below is implemented and checked.
constexpr bool HARDWARE_CONFIGURED = false;

// TODO: replace with actual maze width/height in cells.
constexpr int MAZE_WIDTH  = 8;
constexpr int MAZE_HEIGHT = 8;

// TODO: replace with actual start pose and designated finish cell.
constexpr int START_X = 0;
constexpr int START_Y = 0;
constexpr int START_HEADING = 0;  // 0=N, 1=E, 2=S, 3=W
constexpr int GOAL_X = MAZE_WIDTH - 1;
constexpr int GOAL_Y = MAZE_HEIGHT - 1;

// Confirmed from the handwritten pinout.
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;

// TODO: confirm the PCA9685 board address and motor-driver wiring.
// These four PCA channels are only placeholders for the four motor inputs.
constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint8_t CH_LEFT_FORWARD  = 0;
constexpr uint8_t CH_LEFT_REVERSE  = 1;
constexpr uint8_t CH_RIGHT_FORWARD = 2;
constexpr uint8_t CH_RIGHT_REVERSE = 3;

// TODO: replace after identifying the motor driver and tuning the rover.
constexpr int MOTOR_PWM_FREQUENCY_HZ = 1000;
constexpr int MOTOR_MAX_PWM = 3000;  // PCA9685 range is 0..4095.

// TODO: choose costs using measured movement times.
// Keep all costs in the same units.
constexpr int COST_PER_CELL = 100;
constexpr int COST_TURN_90 = 300;
constexpr int COST_TURN_180 = 600;
constexpr int COST_UNKNOWN_EDGE = 150;

// Challenge timeout from the attached rulebook: 5 minutes.
constexpr unsigned long CHALLENGE_TIMEOUT_MS = 300000UL;

constexpr int STATE_COUNT = MAZE_WIDTH * MAZE_HEIGHT * 4;
constexpr int INF_COST = 0x3fffffff;

// ============================================================
// SENSOR PLACEHOLDERS
// Three VL53L0X modules share the default I2C address.
// TODO: implement sensor initialization using XSHUT pins or an I2C mux.
// TODO: fill in the actual XSHUT pins or mux address/channels.
// GPIO 9 and 10 are not valid choices on the ESP32-WROOM-32.
// ============================================================

constexpr int XSHUT_LEFT_PIN  = -1;  // TODO
constexpr int XSHUT_FRONT_PIN = -1;  // TODO
constexpr int XSHUT_RIGHT_PIN = -1;  // TODO

constexpr int SENSOR_WALL_THRESHOLD_MM = 180;  // TODO: measure/tune

Adafruit_PWMServoDriver pwm(PCA9685_ADDRESS);
Adafruit_VL53L0X tofLeft;
Adafruit_VL53L0X tofFront;
Adafruit_VL53L0X tofRight;

enum Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

struct WallReadings {
    bool valid;
    bool left;
    bool front;
    bool right;
};

bool hardwareReady = false;
bool missionComplete = false;
bool missionFault = false;
unsigned long missionStartMs = 0;

int robotX = START_X;
int robotY = START_Y;
Direction robotHeading = static_cast<Direction>(START_HEADING);

// Maze map: each cell edge records whether it is known and whether it is a wall.
bool wallKnown[MAZE_WIDTH][MAZE_HEIGHT][4] = {};
bool wallPresent[MAZE_WIDTH][MAZE_HEIGHT][4] = {};

struct PlannerData {
    int distance[STATE_COUNT];
    int previous[STATE_COUNT];
    bool settled[STATE_COUNT];
};

// ============================================================
// REQUIRED HARDWARE HOOKS
// These are deliberately fail-closed placeholders.
// Implement them for the actual motor driver, sensors, odometry,
// and physical emergency-stop circuit before enabling the robot.
// ============================================================

bool initializeToFSensors() {
    // TODO:
    // 1. Configure XSHUT pins OR initialize the actual I2C multiplexer.
    // 2. Bring sensors up one at a time.
    // 3. Assign a unique I2C address to each sensor if using XSHUT.
    // 4. Confirm each sensor returns sensible distance readings.
    return false;
}

WallReadings readWallSensors() {
    // TODO: read all three VL53L0X sensors and convert distances to walls.
    // Return valid=false if any required sensor read fails.
    return { false, false, false, false };
}

bool emergencyStopReleased() {
    // TODO: read the physical, accessible emergency-stop circuit.
    // Fail closed until this input is implemented.
    return false;
}

bool turnOneQuarter(bool clockwise) {
    // TODO: command the drive motors to make a calibrated 90-degree turn.
    // Use encoders/gyro if available; return true only when complete.
    (void)clockwise;
    return false;
}

bool driveForwardOneCell() {
    // TODO: drive exactly one cell using encoder/odometry/landmark feedback.
    // Do not use a guessed delay as the final cell-distance controller.
    return false;
}

void stopMotors() {
    // Set every PCA motor-driver input low.
    if (hardwareReady) {
        pwm.setPWM(CH_LEFT_FORWARD, 0, 0);
        pwm.setPWM(CH_LEFT_REVERSE, 0, 0);
        pwm.setPWM(CH_RIGHT_FORWARD, 0, 0);
        pwm.setPWM(CH_RIGHT_REVERSE, 0, 0);
    }
}

void setMotorPair(uint8_t forwardChannel, uint8_t reverseChannel, int speed) {
    speed = constrain(speed, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);

    if (speed > 0) {
        pwm.setPWM(forwardChannel, 0, speed);
        pwm.setPWM(reverseChannel, 0, 0);
    } else if (speed < 0) {
        pwm.setPWM(forwardChannel, 0, 0);
        pwm.setPWM(reverseChannel, 0, -speed);
    } else {
        pwm.setPWM(forwardChannel, 0, 0);
        pwm.setPWM(reverseChannel, 0, 0);
    }
}

void driveMotors(int leftSpeed, int rightSpeed) {
    if (!hardwareReady || missionFault) return;

    setMotorPair(CH_LEFT_FORWARD, CH_LEFT_REVERSE, leftSpeed);
    setMotorPair(CH_RIGHT_FORWARD, CH_RIGHT_REVERSE, rightSpeed);
}

// ============================================================
// MAZE MAP
// ============================================================

int nextX(int x, Direction d) {
    if (d == EAST) return x + 1;
    if (d == WEST) return x - 1;
    return x;
}

int nextY(int y, Direction d) {
    if (d == NORTH) return y + 1;
    if (d == SOUTH) return y - 1;
    return y;
}

Direction opposite(Direction d) {
    return static_cast<Direction>((static_cast<int>(d) + 2) % 4);
}

bool insideMaze(int x, int y) {
    return x >= 0 && x < MAZE_WIDTH && y >= 0 && y < MAZE_HEIGHT;
}

void setWall(int x, int y, Direction d, bool exists) {
    if (!insideMaze(x, y)) return;

    wallKnown[x][y][d] = true;
    wallPresent[x][y][d] = exists;

    int nx = nextX(x, d);
    int ny = nextY(y, d);

    if (insideMaze(nx, ny)) {
        Direction reverse = opposite(d);
        wallKnown[nx][ny][reverse] = true;
        wallPresent[nx][ny][reverse] = exists;
    }
}

void initializeBoundaryWalls() {
    for (int x = 0; x < MAZE_WIDTH; ++x) {
        setWall(x, 0, SOUTH, true);
        setWall(x, MAZE_HEIGHT - 1, NORTH, true);
    }

    for (int y = 0; y < MAZE_HEIGHT; ++y) {
        setWall(0, y, WEST, true);
        setWall(MAZE_WIDTH - 1, y, EAST, true);
    }
}

void updateMapFromSensors(const WallReadings& r) {
    if (!r.valid) return;

    Direction left =
        static_cast<Direction>((static_cast<int>(robotHeading) + 3) % 4);
    Direction right =
        static_cast<Direction>((static_cast<int>(robotHeading) + 1) % 4);

    setWall(robotX, robotY, left, r.left);
    setWall(robotX, robotY, robotHeading, r.front);
    setWall(robotX, robotY, right, r.right);
}

// ============================================================
// HEADING-AWARE DIJKSTRA
// Unknown edges are tentatively considered open with an added
// uncertainty cost. The rover checks the edge with its sensors
// before moving and replans if it finds a wall.
// ============================================================

int stateId(int x, int y, Direction heading) {
    return ((y * MAZE_WIDTH + x) * 4) + static_cast<int>(heading);
}

void decodeState(int id, int& x, int& y, Direction& heading) {
    heading = static_cast<Direction>(id % 4);
    int cell = id / 4;
    x = cell % MAZE_WIDTH;
    y = cell / MAZE_WIDTH;
}

int turnCost(Direction from, Direction to) {
    int diff = abs(static_cast<int>(from) - static_cast<int>(to));
    if (diff == 0) return 0;
    if (diff == 2) return COST_TURN_180;
    return COST_TURN_90;
}

void runDijkstra(PlannerData& p) {
    for (int i = 0; i < STATE_COUNT; ++i) {
        p.distance[i] = INF_COST;
        p.previous[i] = -1;
        p.settled[i] = false;
    }

    int start = stateId(robotX, robotY, robotHeading);
    p.distance[start] = 0;

    for (int iteration = 0; iteration < STATE_COUNT; ++iteration) {
        int current = -1;
        int best = INF_COST;

        for (int i = 0; i < STATE_COUNT; ++i) {
            if (!p.settled[i] && p.distance[i] < best) {
                best = p.distance[i];
                current = i;
            }
        }

        if (current < 0) break;
        p.settled[current] = true;

        int x, y;
        Direction heading;
        decodeState(current, x, y, heading);

        for (int d = 0; d < 4; ++d) {
            Direction moveDir = static_cast<Direction>(d);
            int nx = nextX(x, moveDir);
            int ny = nextY(y, moveDir);

            if (!insideMaze(nx, ny)) continue;
            if (wallKnown[x][y][moveDir] && wallPresent[x][y][moveDir]) {
                continue;
            }

            int uncertainty =
                wallKnown[x][y][moveDir] ? 0 : COST_UNKNOWN_EDGE;

            int next = stateId(nx, ny, moveDir);
            int candidate = p.distance[current] +
                            turnCost(heading, moveDir) +
                            COST_PER_CELL +
                            uncertainty;

            if (candidate < p.distance[next]) {
                p.distance[next] = candidate;
                p.previous[next] = current;
            }
        }
    }
}

int bestGoalState(const PlannerData& p) {
    int bestState = -1;

    for (int d = 0; d < 4; ++d) {
        int candidate = stateId(GOAL_X, GOAL_Y, static_cast<Direction>(d));
        if (bestState < 0 ||
            p.distance[candidate] < p.distance[bestState]) {
            bestState = candidate;
        }
    }

    if (bestState < 0 || p.distance[bestState] == INF_COST) return -1;
    return bestState;
}

int firstDirectionToward(const PlannerData& p, int targetState) {
    int start = stateId(robotX, robotY, robotHeading);
    if (targetState == start) return -1;

    int cursor = targetState;
    while (p.previous[cursor] != start) {
        cursor = p.previous[cursor];
        if (cursor < 0) return -1;
    }

    int x, y;
    Direction firstDirection;
    decodeState(cursor, x, y, firstDirection);
    return static_cast<int>(firstDirection);
}

// ============================================================
// ONE CONTINUOUS AUTONOMOUS SOLVE
// ============================================================

bool turnTo(Direction target) {
    int diff = (static_cast<int>(target) -
                static_cast<int>(robotHeading) + 4) % 4;

    if (diff == 1) {
        if (!turnOneQuarter(true)) return false;
    } else if (diff == 3) {
        if (!turnOneQuarter(false)) return false;
    } else if (diff == 2) {
        if (!turnOneQuarter(true)) return false;
        if (!turnOneQuarter(true)) return false;
    }

    robotHeading = target;
    return true;
}

void failSafeStop(const char* reason) {
    stopMotors();
    missionFault = true;
    Serial.print("STOP: ");
    Serial.println(reason);
}

void solveOneStep() {
    if (!emergencyStopReleased()) {
        failSafeStop("emergency stop is pressed or not configured");
        return;
    }

    WallReadings readings = readWallSensors();
    if (!readings.valid) {
        failSafeStop("ToF sensor readings invalid or not configured");
        return;
    }

    updateMapFromSensors(readings);

    if (robotX == GOAL_X && robotY == GOAL_Y) {
        stopMotors();
        missionComplete = true;
        Serial.println("Finish reached.");
        return;
    }

    PlannerData planner;
    runDijkstra(planner);

    int goalState = bestGoalState(planner);
    if (goalState < 0) {
        failSafeStop("no route to finish in current map");
        return;
    }

    int nextDirection = firstDirectionToward(planner, goalState);
    if (nextDirection < 0) {
        failSafeStop("planner did not return a next move");
        return;
    }

    Direction moveDir = static_cast<Direction>(nextDirection);

    if (!turnTo(moveDir)) {
        failSafeStop("turn controller did not confirm turn completion");
        return;
    }

    // Re-read after facing the planned opening; never drive into a detected wall.
    readings = readWallSensors();
    if (!readings.valid) {
        failSafeStop("ToF sensor read failed before moving");
        return;
    }

    updateMapFromSensors(readings);

    if (readings.front) {
        Serial.println("Wall found on planned edge; replanning.");
        stopMotors();
        return;
    }

    int destinationX = nextX(robotX, moveDir);
    int destinationY = nextY(robotY, moveDir);

    if (!insideMaze(destinationX, destinationY)) {
        failSafeStop("planned move leaves configured maze bounds");
        return;
    }

    setWall(robotX, robotY, moveDir, false);

    if (!driveForwardOneCell()) {
        failSafeStop("cell movement was not confirmed");
        return;
    }

    robotX = destinationX;
    robotY = destinationY;

    Serial.printf(
        "Cell=(%d,%d), heading=%d, time=%lu ms\n",
        robotX,
        robotY,
        static_cast<int>(robotHeading),
        millis() - missionStartMs
    );
}

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    initializeBoundaryWalls();

    if (!HARDWARE_CONFIGURED) {
        Serial.println("Maze sketch loaded, but hardware is disabled.");
        Serial.println("Fill in wiring, ToF, motor, movement, and E-stop hooks first.");
        return;
    }

    if (!pwm.begin()) {
        Serial.println("PCA9685 was not found.");
        return;
    }

    // Note: the PCA9685 has one shared PWM frequency for all channels.
    // Its motor PWM setting may not suit hobby servos at the same time.
    pwm.setPWMFreq(MOTOR_PWM_FREQUENCY_HZ);

    if (!initializeToFSensors()) {
        Serial.println("ToF initialization failed; motors remain stopped.");
        return;
    }

    hardwareReady = true;
    missionStartMs = millis();

    Serial.println("Autonomous maze solve started.");
}

void loop() {
    if (!hardwareReady || missionComplete || missionFault) {
        stopMotors();
        delay(100);
        return;
    }

    if (millis() - missionStartMs >= CHALLENGE_TIMEOUT_MS) {
        failSafeStop("5-minute challenge time limit reached");
        return;
    }

    solveOneStep();
}