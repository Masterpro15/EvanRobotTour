#include <TektiteRotEv.h>

RotEv rotev;
double vErr1, vErr2;
double lt0, ls0, rt0, rs0;

double kP = 0.1;
double delayer_amt;

void setup() {
  rotev.begin();
  Serial.println("Hello, RotEv!");  // rotev.begin() automatically starts Serial
                                    // at 115200 baud
}
// --- LEFT globals ---
double lastAngL = 0, totalAngL = 0;
bool   initL = false;

double lastDL = 0;
unsigned long lastTL = 0;
bool vInitL = false;

// Distance since reset (cm), with wraparound handled
double d1() {
  double curr = rotev.enc1AngleDegrees();   // 0..360
  if (!initL) { lastAngL = curr; initL = true; }

  double delta = curr - lastAngL;
  if (delta > 180)  delta -= 360;           // 360 -> 0 forward
  if (delta < -180) delta += 360;           // 0 -> 360 backward

  totalAngL += delta;                        // continuous degrees
  lastAngL = curr;

  return (totalAngL / 360.0) * (PI * 4.175); // cm
}

// // Velocity (cm/s), no extra encoder reads
// double vL() {
//   double dnow = d1();                        // update distance once
//   unsigned long t = micros();
//   if (!vInitL) { vInitL = true; lastDL = dnow; lastTL = t; return 0.0; }

//   double vel = (dnow - lastDL) / ((t - lastTL) / 1e6);
//   lastDL = dnow; lastTL = t;
//   return vel;
// }

// Use a boolean variable to trigger the GO functionality only when the button
// is released
bool goPressed = false;

// Variable to see if the GO functionality is currently active
bool going = false;

// double degL, degR;
// double d1(){
//   degL += rotev.enc1AngleDegrees();
//   return PI * 4.175 * (degL / 360);
  
// }
double vL()
{
  double d = d1();
  double velocity = (d - ls0) / (micros() - lt0) * 1000000;
  ls0 = d;
  lt0 = micros();
  return velocity;
 }
// double d2(){
//   degR += rotev.enc1AngleDegrees();
//   return PI * 4.175 * (degR / 360);
// }
double vR()
{
  double d = d2();
    double velocity = (d - rs0) / (micros() - rt0) * 1000000;
  rs0 = d;
  rt0 = micros();
  return velocity;
}


// --- RIGHT globals ---
double lastAngR = 0, totalAngR = 0;
bool   initR = false;

double lastDR = 0;
unsigned long lastTR = 0;
bool vInitR = false;

double d2() {
  double curr = rotev.enc1AngleDegrees();   // NOTE: enc2 for right
  if (!initR) { lastAngR = curr; initR = true; }

  double delta = curr - lastAngR;
  if (delta > 180)  delta -= 360;
  if (delta < -180) delta += 360;

  totalAngR += delta;
  lastAngR = curr;

  return (totalAngR / 360.0) * (PI * 4.175);
}

// double vR() {
//   double dnow = d2();
//   unsigned long t = micros();
//   if (!vInitR) { vInitR = true; lastDR = dnow; lastTR = t; return 0.0; }

//   double vel = (dnow - lastDR) / ((t - lastTR) / 1e6);
//   lastDR = dnow; lastTR = t;
//   return vel;
// }
























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
    
  }
}







//im so sorry jay for stealing some of your code + logic, i dont know hopw to do a motion profile 
void foward(double distance, double time){
  float vBat = rotev.getVoltage();
  //update gryo + encoder
  double t0 = micros(); // Start time in microseconds
  double delta_T = time;
  double delta_T_us = delta_T * 1e6; // Convert delta_T from seconds to microseconds
  double motor1PWM = 0.1; //set to the min duty cycle
  double motor2PWM = 0.1;
  
  double velocity_setpoint = 0;
  double elapsed_time;
  while (true)
  {
        elapsed_time = micros() - t0; // Elapsed time in microseconds
    //update gyro and encoders again, also UPDATE BATTERY VOLTAGE
    vBat = rotev.getVoltage();
    if (d1() >= distance)
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


    vErr1 = velocity_setpoint - vL();
    vErr2 = velocity_setpoint - vR();//look above;

    motor1PWM += kP * vErr1;
    motor2PWM += kP * vErr2;

    
    // Constrain PWM values to valid range
    motor1PWM = constrain(motor1PWM, 0.1,  1.0f);
    motor2PWM = constrain(motor2PWM, 0.1, 1.0);

    // Set motor speeds
    rotev.motorWrite1(motor1PWM ); 
    rotev.motorWrite2(motor2PWM ); 
  
    Serial.println(motor1PWM);









     //hie
  }
    rotev.motorWrite1(0); 
    rotev.motorWrite2(0); 
    Serial.println("Done");
    going = false; //change latter
    delay(10 + delayer_amt); // Small delay to ensure stop
    //reset the stuff here
}

void update(){

}