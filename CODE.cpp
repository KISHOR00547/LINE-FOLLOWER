// ================= MOTOR PINS =================
#define AIN1 18
#define AIN2 19
#define PWMA 21
#define BIN1 22
#define BIN2 23
#define PWMB 5
#define STBY 15
int a = 0;
#define MIN_PWM 85   // prevent stall

// ================= SENSOR PINS =================
int sensorPins[10] = {
  39,34,35,32,   // LEFT (4)
  36,4,          // CENTER (2)
  14,27,26,25    // RIGHT (4)
};

// ================= THRESHOLDS =================
int blackThreshold[10] = {
  4050,4050,4050,4050,
  4050,4050,
  4050,4050,4050,4050
};

// ================= SENSOR WEIGHTS =================
float weights[10] = {
  -6, -4.5, -3, -1.5,
  -0.4, 0.4,
   1.5, 3, 4.5, 6
};

// ================= PID GAINS =================
float Kp_center = 0.5;
float Kp_high   = 1.0;
float Kd_center = 1.6;
float Kd_high   = 3.4;

float lastError = 0;
float lastCorrection = 0;
float filteredDErr = 0;

// ================= SPEED SETTINGS =================
int baseSpeed   = 150;
int boostSpeed  = 180;
int turnSpeed   = 150;
int snapSpeed   = 245;
int searchSpeed = 60;
int softTurnMax = 80;
int maxSpeed    = 255;

// ================= MEMORY =================
int lastSeenDir = 0;
int lastSeenCnt = 0;

// ================= ACUTE TURN =================
int acuteHoldCnt = 0;
int acuteDir = 0;
int confidenceCnt = 0;

// ================= MOTOR HELPERS =================
void motorForward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void motorTurnLeft(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void motorTurnRight(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void motorBack() {
  motorTurnLeft(snapSpeed);
  delay(350);
}
void stopMotor(){
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

}

// ================= SETUP =================
void setup() {

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  motorForward();
}

// ================= LOOP =================
void loop() {
  a=analogRead(33);

  bool left=false, center=false, right=false;
  bool extleft=false;
  bool extright=false;
  bool centreleft=false;
  bool centreright=false;

  float posSum = 0;
  int active = 0;

  // -------- SENSOR READ --------
  for (int i = 0; i < 10; i++) {

    int raw = analogRead(sensorPins[i]);

    if (raw > blackThreshold[i]) {

      posSum += weights[i];
      active++;

      // CENTER
      if (i == 4 || i == 5) {
        center = true;
      }

      // LEFT
      else if (i <= 3) {
        left = true;
        lastSeenDir = -1;
        lastSeenCnt++;
        if(i==0 || i==1){
          extleft=true;
        }

        if (i ==3 || a > 4000 || i==2) {
          centreleft = true;
        }
      }

      // RIGHT
      else if (i >= 6) {
        right = true;
        lastSeenDir = +1;
        lastSeenCnt++;
        if(i==9 || i==8){
          extright=true;
        }

        if (i ==6 || i==7  ) {
          centreright = true;
        }
      }
    }
  }

  if (active == 0 && lastSeenCnt > 0) lastSeenCnt--;

  // ================= ACUTE DETECTION =================
  if (!center && active <= 3) {

    if (left && !right){
      confidenceCnt++;
      acuteDir = -1;
    }

    else if (right && !left) {
      confidenceCnt++;
      acuteDir = +1;
    }

  } else confidenceCnt = 0;

  if (confidenceCnt >= 2) {
    acuteHoldCnt = 6;
    confidenceCnt = 0;
  }

  // ================= ACUTE TURN =================
  if (acuteHoldCnt > 0) {

    acuteHoldCnt--;

    if (acuteDir < 0)
      motorTurnLeft(snapSpeed+10);
    else
      motorTurnRight(snapSpeed+10);

    if (center) {

      acuteHoldCnt = 0;
      lastError = 0;
      lastCorrection = 0;
      filteredDErr = 0;

      motorForward();
      
    }

    return;
  }

  // ================= LINE LOST =================
  if (active == 0) {

    int recover = searchSpeed + lastSeenCnt * 6;

    if (lastSeenDir < 0)
      motorTurnLeft(recover);
    else
      motorTurnRight(recover);

    return;
  }

  if (extleft && extright && center) {
    motorForward();
    return;
  }

 
 // ================= STRONG RIGHT PRIORITY =================
 if (extright)
{
 

  lastError = 0;
  lastCorrection = 0;
  filteredDErr = 0;

  motorTurnRight(snapSpeed);

  return;
}
if (extright && center)
{
 

  lastError = 0;
  lastCorrection = 0;
  filteredDErr = 0;

  motorTurnRight(snapSpeed);

  return;
}


if (centreright)
{
 

  lastError = 0;
  lastCorrection = 0;
  filteredDErr = 0;

  motorTurnRight(snapSpeed);

  return;
}
  // ================= PID FOLLOW =================
  if (center) {

    float error = posSum / active;

    if (abs(error) < 0.12)
      error = 0;

    float dErr = error - lastError;
    lastError = error;

    filteredDErr = 0.7 * filteredDErr + 0.3 * dErr;
    filteredDErr = constrain(filteredDErr,-0.6,0.6);

    bool hard = abs(error) > 0.7;

    float Kp = hard ? Kp_high : Kp_center;
    float Kd = hard ? Kd_high : Kd_center;

    int speed = hard ? turnSpeed : boostSpeed;

    float correction = (Kp * error) + (Kd * filteredDErr);

    correction = constrain(correction,lastCorrection-30,lastCorrection+30);
    lastCorrection = correction;

    if (abs(error) > 0.4) {

      float e = constrain(abs(error),0.0f,1.0f);
      float softTurn = softTurnMax * e;

      if (correction > 0)
        correction += softTurn;
      else
        correction -= softTurn;
    }

    correction = constrain(correction,-255,255);

    motorForward();

    analogWrite(PWMA, constrain(speed - correction, MIN_PWM, maxSpeed));
    analogWrite(PWMB, constrain(speed + correction, MIN_PWM, maxSpeed));

    return;
  }

  // ================= STRAIGHT =================
  motorForward();

  analogWrite(PWMA, baseSpeed);
  analogWrite(PWMB, baseSpeed);
  delay(20);
}
