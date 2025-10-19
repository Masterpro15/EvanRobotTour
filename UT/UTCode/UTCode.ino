#include <TektiteRotEv.h>
RotEv rotev;
double vErr1, vErr2;
double lt0, ls0, rt0, rs0;

double kP = 0.1;
double delayer_amt;
const float CM_PER_RAD = (2.375f * 2.54f) / 2.0f;  

// +1 or -1 so that "forward" makes total_cm INCREASE
 const int ENC_L = 1;   // set after a quick test
 const int ENC_R = -1;   // set after a quick test



void setup() {
  rotev.begin();
  Serial.println("Hello, RotEv!");  // rotev.begin() automatically starts Serial
                                    // at 115200 baud
}

bool goPressed = false;

// Variable to see if the GO functionality is currently active
bool going = false;

float unwrapAngle(float delta) {
  if (delta > M_PI) {
    delta -= 2 * M_PI;
  } else if (delta < -M_PI) {
    delta += 2 * M_PI;
  }
  return delta;
}

double vL(double d)
{
  unsigned long now = micros(), dt = now - lt0;
  if (dt < 2000) return 0.0;             // ignore <2ms


  double velocity = (d - ls0) / (dt) * 1000000;
  ls0 = d;
  lt0 = micros();
  return velocity;
}
double vR(double d)
{
  unsigned long now = micros(), dt = now - rt0;
  if (dt < 2000) return 0.0;             // ignore <2ms


  double velocity = (d - rs0) / (dt) * 1000000;
  rs0 = d;
  rt0 = micros();
  return velocity;
}









// Main loop


void loop() {
  // Handle button presses
  if (rotev.goButtonPressed()) {
    goPressed = true;

    rotev.ledWrite(0.0f, 0.1f, 0.0f);  // 10% green
  } else if (goPressed && !rotev.goButtonPressed()) {
    goPressed = false;

    // Trigger the GO functionality
    going = true;
    rotev.motorEnable(true);  // Enable the motor drivers
  } else if (rotev.stopButtonPressed()) {
    going = false;
    rotev.motorEnable(false);  // Disable the motor drivers
    rotev.ledWrite(0.1f, 0.0f, 0.0f);  // 10% red, since stop is pressed
  } else {
    rotev.ledWrite(0.0f, 0.0f, 0.1f); 
  }

  // Motor writes
  if (going) {
    foward(50, 5);
    going = false;
  }
}







//sorry
void foward(double distance, double time){
  float angleL = 0;
  float angleR = 0;
  float delta1 = 0;
  float delta2 = 0;
  float prevAngleL = rotev.enc1Angle();
  float prevAngleR = rotev.enc2Angle();
  float distanceL = 0;
  float distanceR = 0;
  float vBat = rotev.getVoltage();
  //update gryo + encoder
  double t0 = micros(); // Start time in microseconds
  double delta_T = time;
  double delta_T_us = delta_T * 1e6; // Convert delta_T from seconds to microseconds
  double power1 = 0; //set to the min duty cycle
  double power2 = 0;
  float roDistance = 0;
  double velocity_setpoint = 0;
  double elapsed_time;
  float min = 0;
  double cumL = 0.0, cumR = 0.0;  // cumulative distances (cm)
double total_cm = 0.0;          // robot forward distance (cm)
// init velocity baselines
lt0 = micros(); ls0 = cumL;
rt0 = micros(); rs0 = cumR;
  while (true)
  {
    static unsigned long last = micros();
if (micros() - last < 5000) continue;  // run loop every ~5ms
last = micros();

    if (rotev.stopButtonPressed()) {
  rotev.motorWrite1(0.0f);
  rotev.motorWrite2(0.0f);
  rotev.motorEnable(false);
  going = false;
  Serial.println("<< STOP pressed >>");
  // optional: wait until button released so you don't instantly restart
  while (rotev.stopButtonPressed()) { delay(10); }
  return; // abort foward() immediately
}

    elapsed_time = micros() - t0; // Elapsed time in microseconds
    //update gyro and encoders again, also UPDATE BATTERY VOLTAGE
    angleL = rotev.enc1Angle();
    angleR = rotev.enc2Angle();
    delta1 = unwrapAngle(angleL-prevAngleL);
    delta2 = unwrapAngle(angleR-prevAngleR);
    prevAngleL = angleL;
    prevAngleR = angleR;
    vBat = rotev.getVoltage();
    distanceL = ENC_L * delta1 * CM_PER_RAD;
    distanceR = ENC_R * delta2 * CM_PER_RAD;
  cumL += distanceL;
  cumR += distanceR;

    



  

  // cumulative robot distance
  total_cm = 0.5 * (cumL + cumR);
    if (total_cm >= distance)
    {
      break;
    }

     // Determine velocity setpoint based on elapsed time
    if (elapsed_time <= delta_T_us / 4)
    {
      // Acceleration phase
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * (elapsed_time / 1e6);
    }
    else if (elapsed_time <= 3 * delta_T_us / 4)
    {
      // Constant velocity phase
      velocity_setpoint = (4.0 * distance) / (3.0 * delta_T);
    }
    else
    {
      // Deceleration phase
      double t_dec = elapsed_time - 3 * delta_T_us / 4;
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4) - t_dec / 1e6);
    }


    vErr1 = velocity_setpoint - vL(cumL);
    vErr2 = velocity_setpoint - vR(cumR);//look above;
    power1 += kP * vErr1;
    power2 += kP * vErr2;
  
  
     power1 = constrain(power1, -0.3f, 0.3f);
     power2 = constrain(power2, -0.3f, 0.3f);
    if (fabs(power1) < 0.05) power1 = 0.0;
  else if (power1 > 0 && power1 < 0.08) power1 = 0.08;
  else if (power1 < 0 && power1 > -0.08) power1 = -0.08;
  if (fabs(power2) < 0.05) power2 = 0.0;
  else if (power2 > 0 && power2 < 0.08) power2 = 0.08;
  else if (power2 < 0 && power2 > -0.08) power2 = -0.08;
    rotev.motorWrite1( power1); 
    rotev.motorWrite2(power2); 

    Serial.print("total="); Serial.print(total_cm, 2);
  Serial.print(" cumL="); Serial.print(cumL, 2);
  Serial.print(" cumR="); Serial.print(cumR, 2);
  Serial.print(" dutyL=");  Serial.print(power1, 3);
  Serial.print(" dutyR=");  Serial.println(power2, 3);


  delay(1);
  }
    rotev.motorWrite1(0); 
    rotev.motorWrite2(0); 
    Serial.println("Done");
    going = false; //change latter
    delay(10 + delayer_amt); 
    //reset the stuff here
}



