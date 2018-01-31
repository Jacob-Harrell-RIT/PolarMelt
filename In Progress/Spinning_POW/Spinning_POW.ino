#include "SBUS.h"
#include <Servo.h>

// Maximum and Minimum throttles used
const int minthrottle=172;
const int maxthrottle=1811;
// The Maximum and Minimum values written to the DAC
const int dacmin = 0;
const int dacmax = 4096;
// Constants for converting the accelerometer readings
const float maxAccel = 4095.0f;
const float maxG = 250.0f;   // 250g
const float oneG = 980.665f; // 980.665cm/s^2
const float accelRad = 2.5f; // 2.5cm

// Serial Bus for the Reciever
SBUS x8r(Serial2);
// PPM to send to the Motor (in microseconds)
int motorPPM;
// The Motors of the Robot
Servo rightMotor;
Servo leftMotor;
// Reciever input - Data channels, signal lost failsafe, lost data flags
uint16_t channels[16];
uint8_t failSafe;
uint16_t lostFrames;
// Interval Interrupt to read the accelerometers
IntervalTimer accelTimer;
// The current heading of the robot
volatile float heading;
// Keep track of the times between accelerometer readings
elapsedMillis dt;

/*
 * setup()
 * Perform a setup on the robot to make sure everything
 * is enabled
 */
void setup() {
  // Start the serial output for debugging
  Serial.begin(115200);
  // Start up the reciever
  x8r.begin();
  // Attach the motors to output ports
  rightMotor.attach(23,1000,2000); // Pin 23, min time 1000us, max time 2000us
  leftMotor.attach(23,1000,2000); // Pin 22, min time 1000us, max time 2000us
  // Set the motor speeds to "0"
  rightMotor.writeMicroseconds(1430);
  leftMotor.writeMicroseconds(1430);
  // Start the heading at 90*, our "forward" heading
  heading = PI/2.0f;
  // Set the read resolution of the ADC to 12 bits (0-4096)
  analogReadResolution(12);
  // Set the timer to 0
  dt = 0;
  // Begin the interrupt timer to read on an interval
  accelTimer.begin(readAccel,313);
} // setup()

/*
 * loop()
 * Run the robot
 */
void loop() {
  // Try to read the reciever
  if(x8r.read(&channels[0], &failSafe, &lostFrames)){
    // Debug info
    Serial.print("sensor0 = ");
    Serial.print(channels[2]);
    // Map the input into a readable servo PPM output for the motors
    // Currently the wheels will only spin in a circle (no translation)
    motorPPM = map(channels[2],minthrottle,maxthrottle,1000,2000);
    // Debug the mapped PPM
    Serial.println(motorPPM);
    // Write to the motors
    leftMotor.writeMicroseconds(motorPPM);
    rightMotor.writeMicroseconds(motorPPM);
  } // Reciever Read
} // loop()

/*
 * readAccel()
 * Read the accelerometer on a constant loop, calculating the change in
 * heading since the last reading. In addition, toggle the LEDs to indicate
 * a forward heading with POV
 */
void readAccel() {
  // Read the accelerometers, averaging between the two
  float accel = (analogRead(21) + analogRead(20))/2.0f;
  // Adjust the reading, converting to G's (Shift to -2047.5 to 2047.5, then scale by 250g/2047.5)
  accel = (accel - (maxAccel/2.0f))*(2.0f*maxG/maxAccel);
  // Convert to cm/s^2
  accel = accel*oneG;
  // dtheta = dt*sqrt(ac/r)
  float dtheta = (dt/1000.0f)*sqrt(accel/accelRad);
  // Reset the time since last reading
  dt = 0;
  // Add to the current heading, wrapping around the unit circle
  heading = (heading + dtheta) % (2*PI);
  // Check "forward" bounds to turn LED on/off
  if (heading > PI/4.0f && heading < 3*PI/4.0f) {
    // Turn on the LED
  } else {
    // Turn off the LED
  }
} // readAccel()

