
#include <TektiteRotEv.h>
RotEv rotev;


double kP = 1;
const float CM_PER_RAD = (2.375f * 2.54f) / 2.0f;  

double vErr1, vErr2;

double turnAngle = 0;
double gyroRate = 0;
double gyroOffset = 0;
double lastGyro = 0;


double delayer_amt;

// Misc. distances
double lt0, ls0, rt0, rs0;
double angleL;
double angleR;


bool goPressed = false;

// Variable to see if the GO functionality is currently active
bool going = false;


//ANGLE ENCODER VARIBLES
float delta1 = 0;
float delta2 = 0;

float totalAngleL = 0;
float totalAngleR = 0;

float prevAngleL;
float prevAngleR;

void setup()
{
  rotev.begin();
  Serial.println("Hello, RotEv!");  // rotev.begin() automatically starts Serial
                                    // at 115200 baud
}

void reset()
{
  lastGyro = micros();
  turnAngle = 0;
}

float unwrapAngle(float delta) {
  if (delta > M_PI) {
    delta -= 2 * M_PI;
  } else if (delta < -M_PI) {
    delta += 2 * M_PI;
  }
  return delta;
}

double dL() { return totalAngleL * CM_PER_RAD; }
double dR() { return totalAngleR * CM_PER_RAD; }
double vL()
{
  double vel = (dL() - ls0) / (micros() - lt0) * 1000000;
  ls0 = dL();
  lt0 = micros();
  return vel;
}
double vR()
{
  double vel = (dR() - rs0) / (micros() - rt0) * 1000000;
  rs0 = dR();
  rt0 = micros();
  return vel;
}
void foward(double distance, double time)
{
   updateGyro();  
   prevAngleL = -rotev.enc1Angle();
   prevAngleR = rotev.enc2Angle();
  totalAngleL = 0;
  totalAngleR = 0;

  double t0 = micros(); // Start time in microseconds
  double delta_T = time;
  double delta_T_us = delta_T * 1e6; // Convert delta_T from seconds to microseconds

  double velocity_setpoint = 0; // Initialize the velocity setpoint
  double elapsed_time;

  double power1 = 0.2; //set to the min duty cycle
  double power2 = 0.2;
  

  
  // Main control loop
  while (true)
  {

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
    updateGyro();
    angleL = -rotev.enc1Angle();
    angleR = rotev.enc2Angle();

    delta1 = unwrapAngle(angleL-prevAngleL);
    delta2 = unwrapAngle(angleR-prevAngleR);

    totalAngleL += delta1;
    totalAngleR += delta2;

    prevAngleL = angleL;
    prevAngleR = angleR;
    // Exit condition: Distance has been covered or time has exceeded delta_T
    if (dR() >= distance)
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

    // Update PWM values based on velocity feedback and setpoint
    vErr1 = velocity_setpoint - vL();
    vErr2 = velocity_setpoint - vR();

    power1 += kP * vErr1;
    power2 += kP * vErr2;

    // Constrain PWM values to valid range
    power1 = constrain(power1, 0.5f, 1.0f);
    power2 = constrain(power2, 0.5f, 1.0f);
    Serial.println(-power1);

    rotev.motorWrite1(-power1); 
    rotev.motorWrite2(-power2); 

  }

  // Stop motors at the end
    rotev.motorWrite1(0); 
    rotev.motorWrite2(0);  
    delay(10 + delayer_amt); // Small delay to ensure stop
    //reset t]stuff
      reset();
      prevAngleL = -rotev.enc1Angle();
      prevAngleR = rotev.enc2Angle();
}








// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void updateGyro()
{
  gyroRate = rotev.readYawRateDegrees(); // Returns yaw rate in deg/s
  double currentT = micros();
  double dt = (currentT - lastGyro) / 1e6;;
  lastGyro = currentT;
  turnAngle += gyroRate * dt;
}
double heading()
{
  return turnAngle;
}





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
    rotev.motorWrite1(2.0f); 
    rotev.motorWrite2(1.0f); 
    Serial.println("THE PATH IS DONEEEEED");
    Serial.println(rotev.getVoltage());

    going = false;

  }
}
