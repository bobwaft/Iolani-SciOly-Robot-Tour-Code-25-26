#include <Encoder.h>
#include "DualG2HighPowerMotorShield.h"
#include <AStar32U4.h>  // For B button

DualG2HighPowerMotorShield24v14 md;
AStar32U4ButtonB buttonB;  // Use B button from A-Star library

Encoder encLeft (3, 0);   // D3 (INT0), D0 (INT2)
Encoder encRight(1, 6);   // D1 (INT3), D6 (PCINT?)

const int maxSpeed = 350;
const long targetAdvance45 = 9003.163162;
const int minSpeed = 150;
const int turnMaxSpeed  = 150;
const int turnMinSpeed  = 60;
const float kP = 0.5;

const float motorBias = 0.69;  // Reduce right motor speed 
const long targetAdvance = 6300;
const long targetAdvanceBottle = 6571; // Example value, adjust as needed
const long startTargetAdvance = 4456;
const long startTargetAdvanceBottle = 5000; // Example value, adjust as needed
// Individual turn counts for left and right turns
const long turnCountL_left   = -1800;   // Left turn left wheel NEG
const long turnCountR_left   = 1800*0.5;  // Left turn right wheel POS SIX SEVEN
const long turnCountL_right  = 2050;   // Right turn left wheel POS
const long turnCountR_right  = -2050;  // Right turn right wheel NEG
const long bottleTurnAdjustment = 1.15;

// Individual 45-degree turn counts
const long turn45CountL_left  = turnCountL_left / 2;
const long turn45CountR_left  = turnCountR_left / 2;
const long turn45CountL_right = turnCountL_right / 2;
const long turn45CountR_right = turnCountR_right / 2;
// Individual 180-degree turn counts
const long turn180CountL_left  = turnCountL_left * 2;
const long turn180CountR_left  = turnCountR_left * 2;
const long turn180CountL_right = turnCountL_right * 2;
const long turn180CountR_right = turnCountR_right * 2;
// Alternate turn counts for bottle actions
const long turnCountL_left_bottle   = turnCountL_left * bottleTurnAdjustment;   // Example adjustment
const long turnCountR_left_bottle   = turnCountR_left * bottleTurnAdjustment;
const long turnCountL_right_bottle  = turnCountL_right * bottleTurnAdjustment;
const long turnCountR_right_bottle  = turnCountR_right * bottleTurnAdjustment;
const long turn45CountL_left_bottle  = turn45CountL_left * bottleTurnAdjustment;
const long turn45CountR_left_bottle  = turn45CountR_left * bottleTurnAdjustment;
const long turn45CountL_right_bottle = turn45CountL_right * bottleTurnAdjustment;
const long turn45CountR_right_bottle = turn45CountR_right * bottleTurnAdjustment;
const long turn180CountL_left_bottle  = turn180CountL_left * bottleTurnAdjustment;
const long turn180CountR_left_bottle  = turn180CountR_left * bottleTurnAdjustment;
const long turn180CountL_right_bottle = turn180CountL_right * bottleTurnAdjustment;
const long turn180CountR_right_bottle = turn180CountR_right * bottleTurnAdjustment;

enum ActionType { DRIVE, RIGHTTURN, LEFTTURN , RIGHTTURN45, LEFTTURN45, LEFTTURN180, RIGHTTURN180, BACKWARD };

struct Action {
  ActionType type;
  long target;
  bool bottle;
};

Action actions[] = {
  {DRIVE,startTargetAdvance+targetAdvance,0},
  {RIGHTTURN,0,0},
  {DRIVE,targetAdvance,0},
  {RIGHTTURN,0,0},
  {DRIVE,targetAdvance,1},
  {LEFTTURN,0,1},
  {DRIVE,targetAdvance,1},
  {LEFTTURN,0,1},
  {DRIVE,targetAdvance,1},
  {BACKWARD,targetAdvance,0},
  {LEFTTURN,0,0},
  {DRIVE,targetAdvance,0},
  {RIGHTTURN,0,0},
  {DRIVE,targetAdvance*3,0},
  {RIGHTTURN,0,0},
  {DRIVE,targetAdvance,0},
  {BACKWARD,targetAdvance*3,0},
  {RIGHTTURN,0,0},
  {DRIVE,targetAdvance*2,0},
  {DRIVE,targetAdvance,1},
  {RIGHTTURN,0,1},
  {DRIVE,targetAdvance,1},
  {LEFTTURN,targetAdvance,1},
  {BACKWARD,targetAdvance*3,0}
};

const int numActions = sizeof(actions) / sizeof(actions[0]);
int currentActionIndex = 0;

enum ExecState { WAITING_TO_START, RUNNING, WAITING };
ExecState execState = WAITING_TO_START;
unsigned long waitStartTime = 0;
const unsigned long waitDuration = 200;

bool hasStartedDrive = false;

void setup() {
  pinMode(A2, OUTPUT); digitalWrite(A2, HIGH);
  pinMode(A3, OUTPUT); digitalWrite(A3, HIGH);

  Serial.begin(115200);
  md.init();

  pinMode(2, OUTPUT); digitalWrite(2, HIGH);
  pinMode(4, OUTPUT); digitalWrite(4, HIGH);

  pinMode(3, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);

  encLeft.write(0);
  encRight.write(0);

  md.setM1Speed(0);
  md.setM2Speed(0);
}

void loop() {
  if (execState == WAITING_TO_START) {
    if (buttonB.getSingleDebouncedPress()) {
      delay(1000);
      Serial.println("Button B pressed. Starting robot.");
      execState = RUNNING;
    }
    return;
  }

  if (currentActionIndex >= numActions) {
    md.setSpeeds(0, 0);
    return;
  }

  if (execState == WAITING) {
    if (millis() - waitStartTime >= waitDuration) {
      encLeft.write(0);
      encRight.write(0);
      execState = RUNNING;
    }
    return;
  }

  long L = encLeft.read();
  long R = encRight.read();
  Action current = actions[currentActionIndex];

  switch (current.type) {
    case DRIVE: {
      long driveTarget;
      if (current.bottle) {
       driveTarget = ((current.target)/targetAdvance)*targetAdvanceBottle;
      } else {
        driveTarget = current.target;
      }
      driveToTarget(driveTarget, L, R, true);
      break;
    }
    case BACKWARD: {
      long driveTarget;
      if (current.bottle) {
        driveTarget = ((current.target)/targetAdvance)*targetAdvanceBottle;
      } else {
        driveTarget = current.target;
      }
      driveToTarget(driveTarget, L, R, false);
      break;
    }
    case RIGHTTURN:
      if (current.bottle)
        performTurn(L, R, turnCountL_right_bottle, turnCountR_right_bottle, true);
      else
        performTurn(L, R, turnCountL_right, turnCountR_right, true);
      break;
    case LEFTTURN:
      if (current.bottle)
        performTurn(L, R, turnCountL_left_bottle, turnCountR_left_bottle, false);
      else
        performTurn(L, R, turnCountL_left, turnCountR_left, false);
      break;
    case RIGHTTURN45:
      if (current.bottle)
        performTurn(L, R, turn45CountL_right_bottle, turn45CountR_right_bottle, true);
      else
        performTurn(L, R, turn45CountL_right, turn45CountR_right, true);
      break;
    case LEFTTURN45:
      if (current.bottle)
        performTurn(L, R, turn45CountL_left_bottle, turn45CountR_left_bottle, false);
      else
        performTurn(L, R, turn45CountL_left, turn45CountR_left, false);
      break;
    case LEFTTURN180:
      if (current.bottle)
        performTurn(L, R, turn180CountL_left_bottle, turn180CountR_left_bottle, false);
      else
        performTurn(L, R, turn180CountL_left, turn180CountR_left, false);
      break;
    case RIGHTTURN180:
      if (current.bottle)
        performTurn(L, R, turn180CountL_right_bottle, turn180CountR_right_bottle, true);
      else
        performTurn(L, R, turn180CountL_right, turn180CountR_right, true);
      break;
  }

  delay(50);
}

void driveToTarget(long targetCount, long L, long R, bool forward) {
  long avgCount = (L + R) / 2;
  long distanceTravelled = abs(avgCount);

  float rampFraction = 0.25;
  long rampDistance = targetCount * rampFraction;
  int targetSpeed;

  if (!hasStartedDrive) {
    md.setM1Speed(0);
    md.setM2Speed(0);
    delay(100);
    hasStartedDrive = true;
  }

  // Ramping logic based on absolute progress
  if (distanceTravelled < rampDistance) {
    targetSpeed = map(distanceTravelled, 0, rampDistance, minSpeed, maxSpeed);
  } else if (distanceTravelled > (targetCount - rampDistance)) {
    targetSpeed = map(distanceTravelled, targetCount - rampDistance, targetCount, maxSpeed, minSpeed);
  } else {
    targetSpeed = maxSpeed;
  }

  // Correct error sign for reverse driving
  long err;
  if (forward) {
    err = L - R; // normal
  } else {
    err = R - L; // swap for reverse
  }

  int adjust = constrain(int(kP * err), -targetSpeed, targetSpeed);
  int speedL = constrain(targetSpeed - adjust, 0, maxSpeed);
  int speedR = constrain(targetSpeed + adjust, 0, maxSpeed);

  speedR = int(speedR * motorBias);

  if (!forward) {
    speedL = -speedL;
    speedR = -speedR;
  }

  md.setM1Speed(speedL);
  md.setM2Speed(speedR);

  Serial.print(forward ? "Drive FWD L=" : "Drive BWD L="); Serial.print(L);
  Serial.print(" R="); Serial.print(R);
  Serial.print(" err="); Serial.print(err);
  Serial.print(" sL="); Serial.print(speedL);
  Serial.print(" sR="); Serial.println(speedR);

  if (distanceTravelled >= targetCount) {
    md.setSpeeds(0, 0);
    currentActionIndex++;
    execState = WAITING;
    waitStartTime = millis();
    hasStartedDrive = false;
    Serial.println(forward ? "Forward drive complete." : "Backward drive complete.");
  }
}




void turnInPlaceRight(long tL, long tR) {
  // Use individually adjustable right turn counts
  performTurn(tL, tR, turnCountL_right, turnCountR_right, true);
}

void turnInPlaceLeft(long tL, long tR) {
  // Use individually adjustable left turn counts
  performTurn(tL, tR, turnCountL_left, turnCountR_left, false);
}

void turn45Right(long tL, long tR) {
  performTurn(tL, tR, turn45CountL_right, turn45CountR_right, true);
}

void turn45Left(long tL, long tR) {
  performTurn(tL, tR, turn45CountL_left, turn45CountR_left, false);
}

void turn180Left(long tL, long tR) {
  performTurn(tL, tR, turn180CountL_left, turn180CountR_left, false);
}

void turn180Right(long tL, long tR) {
  performTurn(tL, tR, turn180CountL_right, turn180CountR_right, true);
}

void performTurn(long tL, long tR, long tgtL, long tgtR, bool isRightTurn) {
  float rampFraction = 0.8;
  long totalTargetDistanceL = abs(tgtL);
  long totalTargetDistanceR = abs(tgtR);

  long absL = abs(tL);
  long absR = abs(tR);

  float fracL = min(1.0, float(absL) / totalTargetDistanceL);
  float fracR = min(1.0, float(absR) / totalTargetDistanceR);
  float frac = min(fracL, fracR);

  long rampDistanceL = totalTargetDistanceL * rampFraction;
  long rampDistanceR = totalTargetDistanceR * rampFraction;

  int baseSpeedL = (absL < rampDistanceL) ? map(absL, 0, rampDistanceL, turnMinSpeed, turnMaxSpeed) :
                    (absL > (totalTargetDistanceL - rampDistanceL) ? map(absL, totalTargetDistanceL - rampDistanceL, totalTargetDistanceL, turnMaxSpeed, turnMinSpeed) : turnMaxSpeed);

  int baseSpeedR = (absR < rampDistanceR) ? map(absR, 0, rampDistanceR, turnMinSpeed, turnMaxSpeed) :
                    (absR > (totalTargetDistanceR - rampDistanceR) ? map(absR, totalTargetDistanceR - rampDistanceR, totalTargetDistanceR, turnMaxSpeed, turnMinSpeed) : turnMaxSpeed);

  int targetSpeed = min(baseSpeedL, baseSpeedR);
  long err = tL + tR;
  int adjust = constrain(int(kP * err), 0, 30);

  int leftDir = (tgtL > 0) ? 1 : -1;
  int rightDir = (tgtR > 0) ? 1 : -1;

  int speedL = constrain(targetSpeed - adjust, turnMinSpeed, turnMaxSpeed) * leftDir;
  int speedR = constrain(targetSpeed + adjust, turnMinSpeed, turnMaxSpeed) * rightDir;

  bool leftDone = (tgtL > 0) ? (tL >= tgtL) : (tL <= tgtL);
  bool rightDone = (tgtR > 0) ? (tR >= tgtR) : (tR <= tgtR);

  if (leftDone && rightDone) {
    md.setSpeeds(0, 0);
    currentActionIndex++;
    execState = WAITING;
    waitStartTime = millis();
    Serial.println("Turn complete.");
  } else {
    // NEW LOGIC: If one side is done, slow the other down to turnMinSpeed, not zero
    int finalSpeedL = leftDone  ? turnMinSpeed * leftDir  : speedL;
    int finalSpeedR = rightDone ? turnMinSpeed * rightDir : speedR;

    md.setM1Speed(finalSpeedL);
    md.setM2Speed(finalSpeedR);
  }

  Serial.print(isRightTurn ? "Right" : "Left");
  Serial.print(" Turn L="); Serial.print(tL);
  Serial.print(" R="); Serial.print(tR);
  Serial.print(" sL="); Serial.print(leftDone ? 0 : speedL);
  Serial.print(" sR="); Serial.println(rightDone ? 0 : speedR);
}
