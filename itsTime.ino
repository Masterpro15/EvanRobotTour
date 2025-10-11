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

// Use a boolean variable to trigger the GO functionality only when the button
// is released
bool goPressed = false;

// Variable to see if the GO functionality is currently active
bool going = false;


double d1(){
  double distanceInCm  =  PI * 4.175 / rotev.enc1Angle();
}
double vL()
{
  double velocity = (d1() - ls0) / (micros() - lt0) * 1000000;
  ls0 = d1();
  lt0 = micros();
  return vel;
}
double d2(){
  double distanceInCm  =  PI * 4.175 / rotev.enc2Angle();
}
double vR()
{
  double velocity = (d2() - rs0) / (micros() - rt0) * 1000000;
  rs0 = d2();
  rt0 = micros();
  return vel;
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
    // rotev.motorWrite1(0.5f);  // Set motor 1 speed to 50%

    // float voltage = rotev.getVoltage();  // Returns the voltage of the battery
    // rotev.motorWrite2(4.0f / voltage);   // Set motor 2 voltage to 4V


  }
}

// SECOND CORE: The RP2350 processor has two cores. You can use the FIFO bus to
// transfer data or use global variables. If using global variables you may want
// to use mutexes.
void setup1() {
  // You can put setup code for the second core here
}

void loop2() {
  // I recommend doing sensor fusion on the second core, since this needs to
  // occur at a high rate. In this example it is printing out sensor data.
  float yawRateDeg = rotev.readYawRateDegrees();
  float enc1Angle = rotev.enc1Angle();
  float enc2Angle = rotev.enc2Angle();
  float curr1 = rotev.motorCurr1();
  float curr2 = rotev.motorCurr2();
  float voltage = rotev.getVoltage();

  // If you print in this format, you can use Arduino's Serial monitor to plot
  // the data over time
  Serial.print("yawRate:");
  Serial.print(yawRateDeg);

  Serial.print(",enc1:");
  Serial.print(enc1Angle);
  Serial.print(",enc2:");
  Serial.print(enc2Angle);

  Serial.print(",curr1:");
  Serial.print(curr1);
  Serial.print(",curr2:");
  Serial.print(curr2);

  Serial.print(",voltage:");
  Serial.print(voltage);

  Serial.println();

  delay(50);  // This slows down the printing rate. Note that you would not want
              // a delay in your actual code, since sensor fusion must occur at
              // a high frequency
}








//currently writing logic only
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
    motor1PWM = constrain(left_pwm, 0.1,  1.0f);
    motor2PWM = constrain(right_pwm, 0.1, 1.0f);

    // Set motor speeds
    rotev.motorWrite1(motor1PWM / vbat); 
    rotev.motorWrite2(motor2PWM / vbat); 












  }
    rotev.motorWrite1(0f); 
    rotev.motorWrite2(0f); 

    delay(10 + delayer_amt); // Small delay to ensure stop
    //reset the stuff here
}

void update(){

}