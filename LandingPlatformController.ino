/*
  17HD40005-22B 
  Can run at stepsPerRevolution=400 and rotationsPerSecond=4
  A=1.3
  V=2.4

   - Can run at stepsPerRevolution=200 and rotationsPerSecond=8
    A=1.3
  V=[22..31] -> 31 for smooth usage
   
   - Can run at stepsPerRevolution=200 and rotationsPerSecond=4
    A=1.3
  V=6 (smoothly)
  
  !!! This motor should not run more than 4 rotations/second to avoid losing precision! !!!
  !!! Arduino Motorshield can only handle 18V at its terminals so yikes... !!!

  23HS41-1804S
  Can run at stepsPerRevolution=200 and rotationsPerSecond=4
  A=1.8
  V=19
*/

#include <Stepper.h>
//#include <Math.h>

const int STEPS_PER_REVOLUTION = 200;
const int rotationsPerSecond = 1;
const double MAX_SPEED = STEPS_PER_REVOLUTION * rotationsPerSecond;

const double ZAEHNE = 20;
const double ZAEHNE_ABSTAND = 2;
const double REVOLUTION = (ZAEHNE * ZAEHNE_ABSTAND); 
const double STEP_DISTANCE = REVOLUTION / STEPS_PER_REVOLUTION; //<- How much do we travel per step?

const bool INVERSE_STEPS = false; // doesnt work i think

Stepper stepper(STEPS_PER_REVOLUTION, 12, 13);

const int pmwA = 3;
const int pmwB = 11;

const int brakeA = 9;
const int brakeB = 8;

const int dirA = 12;
const int dirB = 13;

//--- x-axis
const int X_T_HOMER_MIN = 5;
const int X_T_HOMER_MAX = 6;

int X_STEP_COUNTER = -1;
int X_STEP_MAX = -1;


void setup() {
  pinMode(pmwA, OUTPUT);
  pinMode(pmwB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);
  pinMode(A0, INPUT);
 
  pinMode(X_T_HOMER_MIN, INPUT);

  digitalWrite(pmwA, HIGH);
  digitalWrite(pmwB, HIGH);

  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);

  //stepper.setSpeed(((int)(60 * rotationsPerSecond)));

  Serial.begin(115200);
}


bool runLoop = false;
int turns = 10000;
short dir = 1;
int mode = 0;

void loop() {
  
  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);

  if(Serial.available()) {
    String read = Serial.readStringUntil('\n');
  
    if (read != NULL) {
      if (read.startsWith("turn")) {
        double distance = read.substring(5).toDouble();
        long time = millis();
        move(distance, mode, 3);
        Serial.println(millis() - time);
      }
      else if (read.equals("loop")) {
        runLoop = !runLoop;
      } else if(read.equals("step")) {
        stepper.step(1);
      } else if(read.equals("fr")) {
        stepper.step(200);
      } else if(read.equals("-fr")) {
        stepper.step(-200);
      } else if(read.startsWith("mode")) {
        mode = read.substring(5).toInt();
      } else if(read.equals("homing")) {
        homing();
      }
    }   
  }
 

  if (runLoop) {
    move(X_STEP_MAX * STEP_DISTANCE * dir, mode, 2);
    dir *= -1;
  }
}

bool HOMING_MODE = false;
void homing() {
  Serial.println("Start homing");
  HOMING_MODE = true;
  while(digitalRead(X_T_HOMER_MIN) == HIGH) {
    move(-10, 4, 5);
  }
  X_STEP_COUNTER = 0;
  X_STEP_MAX = 0;

  //move(20, 4); //move 80cm
  while(digitalRead(X_T_HOMER_MAX) == HIGH) {
    int steps = move(1, 4, 1);
    X_STEP_MAX += abs(steps);
    Serial.println(X_STEP_MAX);
  }

  X_STEP_COUNTER = X_STEP_MAX;
  HOMING_MODE = false;
  Serial.println(X_STEP_MAX);
  Serial.println(X_STEP_MAX * STEP_DISTANCE);
  Serial.println("End homing");
}

double error = 0;
int move(double distance, int mode, int errfef) {
  
  double rev = (double) distance / (double) STEP_DISTANCE;
  if(INVERSE_STEPS) {
    rev *= -1;
  }
  
  Serial.println("rev: " + (String) rev + ", distance: " + (String) distance + ", STEP_DISTANCE: " + (String) STEP_DISTANCE);
  if(!HOMING_MODE && (X_STEP_COUNTER + rev < 0 || X_STEP_COUNTER + rev > X_STEP_MAX)) {
    Serial.println("Illegal move");
    return -1;
  }

  if(!HOMING_MODE) {
    X_STEP_COUNTER += rev;
  }
  
  double easeIn = .05D;
  double easeOut = .05D;
  double sign = copysignf(1.0, rev);
  rev = abs(rev);
  
  int easeInSteps = (int) (easeIn * rev);
  int easeOutSteps = (int) (easeOut * rev);
  int waySteps = (rev - easeInSteps - easeOutSteps);

  int type = mode;

  long now = millis();
  for(int i = 0; i < rev; i++) {
    int dSpeed;
    if(type == 4) {
      dSpeed = MAX_SPEED;
    } else if(type != 4) {
      if(i < easeInSteps) {
        if(type == 0) {
          dSpeed = easeLinear(i + 1, MAX_SPEED / (double) easeInSteps, 0);
        } else if(type == 1) {
          dSpeed = easeQuad(i + 1, easeInSteps, MAX_SPEED, 0);
        } else if(type == 2) {
          dSpeed = easeSmoothly(i + 1, easeInSteps, MAX_SPEED, false);
        }
      } else if (i > waySteps + easeInSteps) {
        if(type == 0) {
          dSpeed = easeLinear((i + 1) - (easeInSteps + waySteps), -(MAX_SPEED / (double) easeInSteps), MAX_SPEED);
        } else if(type == 1) {
          dSpeed = easeQuad((i + 1) - (easeInSteps + waySteps), easeOutSteps, -MAX_SPEED, MAX_SPEED);
        } else if(type == 2) {
          dSpeed = easeSmoothly((i + 1) - (easeInSteps + waySteps), easeOutSteps, MAX_SPEED, true);
        }
      } else {
          dSpeed = MAX_SPEED;
      }
    }
    stepper.setSpeed(dSpeed);
    stepper.step(1 * sign);
  }
  Serial.println(" Time: " + (String) (millis() - now) + "ms");

  return rev;
}

double easeLinear(double time, double slope, double startValue) {
  return max(10, ceil(time * slope + startValue));
}

double easeQuad(double time, double totalTime, double maxValue, double startValue) {
  time /= totalTime;
  return max(10, ceil(time * time * maxValue) + startValue);
}

double easeSmoothly(double time, double totalTime, double maxValue, bool inverse) {
  time /= totalTime;
  if(inverse) {
    time = 1 - time;
  }
  return max(10, ceil(maxValue * (3 * pow(time, 2) - 2 * pow(time, 3))));
}
