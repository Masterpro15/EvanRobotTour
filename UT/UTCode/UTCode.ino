#include <TektiteRotEv.h>
//test github...
RotEv rotev;
double vErr1, vErr2;
double lt0, ls0, rt0, rs0;

double kP = 0.1;
double delayer_amt;
const float CM_PER_RAD = (2.375f * 2.54f) / 2.0f;  


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
  double velocity = (d - ls0) / (micros() - lt0) * 1000000;
  ls0 = d;
  lt0 = micros();
  return velocity;
}
double vR(double d)
{
  double velocity = (d - rs0) / (micros() - rt0) * 1000000;
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
    break;
  }
  break;
}







//im so sorry jay for stealing some of your code + logic, i dont know hopw to do a motion profile 
void foward(double distance, double time){
  float angleL;
  float angleR;
  float delta1, delta2;
  float prevAngleL,prevAngleR;
  float distanceL, distanceR,
  float vBat = rotev.getVoltage();
  //update gryo + encoder
  double t0 = micros(); // Start time in microseconds
  double delta_T = time;
  double delta_T_us = delta_T * 1e6; // Convert delta_T from seconds to microseconds
  double motor1PWM = 0; //set to the min duty cycle
  double motor2PWM = 0;
  float roDistance = 0;
  double velocity_setpoint = 0;
  double elapsed_time;
  float min = 0;
  while (true)
  {
    elapsed_time = micros() - t0; // Elapsed time in microseconds
    //update gyro and encoders again, also UPDATE BATTERY VOLTAGE
    angleL = rotev.enc1Angle();
    angleR = rotev.enc2Angle();
    delta1 = unwrapAngle(angleL-prevAngleL) * -1.0f;
    delta2 = unwrapAngle(angleR-prevAngleR);
    prevAngleL = angleL;
    prevAngleR = angleR;
    vBat = rotev.getVoltage();
    distanceL = delta1 * CM_PER_RAD;
    distanceR = delta2 * CM_PER_RAD;
    roDistance = (distanceL +distanceR)/2.0f;
    if (roDistance >= distance)
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


    vErr1 = velocity_setpoint - vL(distanceL);
    vErr2 = velocity_setpoint - vR(distanceR);//look above;
    motor1PWM += kP * vErr1;
    motor2PWM += kP * vErr2;



    
    // motor1PWM = constrain(motor1PWM, 0.05f,  1.0f);
    // motor2PWM = constrain(motor2PWM, 0.05f, 1.0f);

    rotev.motorWrite1(motor1PWM *-1.0f); 
    rotev.motorWrite2(motor2PWM *-1.0f); 
  
    Serial.println(roDistance);

  }
    rotev.motorWrite1(0); 
    rotev.motorWrite2(0); 
    Serial.println("Done");
    going = false; //change latter
    delay(10 + delayer_amt); 
    //reset the stuff here
}

