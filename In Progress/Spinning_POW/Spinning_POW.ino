#include "SBUS.h"
#include <Servo.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// THROTTLE
const int THROTTLE_MIN=172;
const int THROTTLE_MAX=1811;
// MOTOR
const int MOTOR_MIN=1000; // Full Reverse
const int MOTOR_OFF=1430; // The motors are off
const int MOTOR_MAX=2000; // Full Forward
// DAC
const int dacmin = 0;
const int dacmax = 4096;
// ACCELERATION
const float ACC_MAX = 4095.0f;
const float G_MAX   = 250.0f;    // 250g
const float G       = 980.665f;  // 980.665cm/s^2
const float ACC_RADIUS = 2.5f;   // 2.5cm
// PORTS
const int ACC_PORT_A  = 20;  // Port 20 for an Accelerator (rotational)
const int ACC_PORT_B  = 21;  // Port 21 for an Accelerator (rotational)
const int ACC_PORT_VERT = 0; // Port ??? for an Accelerator (vertical)
const int LED_PORT_A  = 0;   // Port ??? for an LED Strip
const int LED_CLOCK_A = 0;   // Port ??? for an LED Strip's clock
const int LED_PORT_B  = 0;   // Port ??? for an LED Strip
const int LED_CLOCK_B = 0;   // Port ??? for an LED Strip
// INPUT CHANNELS
const int THROTTLE_CHANNEL = 2; // The channel of the throttle
const int TRANSLATION_CHANNEL = 0; // TODO Get the channel of the translation information
// LED
const int NUM_PIXELS = 19;
const int LED_OFF = 0;
const int LED_ON = 0x00FFFF;
const int LED_ERR = 0xFF0000;
// VERTICAL ORIENTATION
const int VERT_NORM = 2253;
const int VERT_FLIP = 1843;

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
// Setup the AdaFruit LED Strips
Adafruit_DotStar strip_a = Adafruit_DotStar(
  NUM_PIXELS,LED_PORT_A,LED_CLOCK_A,DOTSTAR_BRG);
Adafruit_DotStar strip_b = Adafruit_DotStar(
  NUM_PIXELS,LED_PORT_A,LED_CLOCK_A,DOTSTAR_BRG);

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
  rightMotor.attach(23,MOTOR_MIN,MOTOR_MAX); // Pin 23, min time 1000us, max time 2000us
  leftMotor.attach(22,MOTOR_MIN,MOTOR_MAX); // Pin 22, min time 1000us, max time 2000us
  // Make sure the motors are killed
  rightMotor.writeMicroseconds(MOTOR_OFF);
  leftMotor.writeMicroseconds(MOTOR_OFF);
  // Start the heading at 90*, our "forward" heading
  heading = PI/2.0f;
  // Set the read resolution of the ADC to 12 bits (0-4096)
  analogReadResolution(12);
  // Setup the LEDs, immediately turning them off
  strip_a.begin();
  strip_a.show();
  strip_b.begin();
  strip_b.show();
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
    // Debug the failsafe
    Serial.print(failSafe);
    // If the failsafe has been tripped
    if(failSafe) {
      // Kill the motors
      rightMotor.writeMicroseconds(MOTOR_OFF);
      leftMotor.writeMicroseconds(MOTOR_OFF);
      // Debug the fail state
      Serial.print("Signal Lost!!!");
    } else {
      // Debug info
      Serial.print("sensor0 = ");
      Serial.print(channels[THROTTLE_CHANNEL]);
      // Map the input into a readable servo PPM output for the motors
      // Currently the wheels will only spin in a circle (no translation)
      motorPPM = map(channels[THROTTLE_CHANNEL],THROTTLE_MIN,THROTTLE_MAX,MOTOR_MIN,MOTOR_MAX);
      // Debug the mapped PPM
      Serial.println(motorPPM);
      // Write to the motors
      leftMotor.writeMicroseconds(motorPPM);
      rightMotor.writeMicroseconds(motorPPM);
      // TODO Adjust based on current heading and desired translation
      // Speed of translation depends on strength of joystick movement
      // If zero, both wheels at max speed
      // As the strength increases, the PPM over time changes to either a
      // SINE/COS Wave or a flattened triangle wave
    } // Failsafe trippped?
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
  float accel = (analogRead(ACC_PORT_A) + analogRead(ACC_PORT_B))/2.0f;
  // Adjust the reading, converting to G's (Shift to -2047.5 to 2047.5, then scale by 250g/2047.5)
  accel = (accel - (ACC_MAX/2.0f))*(2.0f*G_MAX/ACC_MAX);
  // Convert to cm/s^2
  accel = accel*G;
  // dtheta = dt*sqrt(ac/r)
  float dtheta = (dt/1000.0f)*sqrt(accel/ACC_RADIUS);
  // Reset the time since last reading
  dt = 0;
  // Add to the current heading, wrapping around the unit circle
  heading = heading + dtheta;
  if (heading >= 2*PI) heading -= 2*PI;
  // Check "forward" bounds to turn LED on/off
  if (heading > PI/4.0f && heading < 3*PI/4.0f) {
    // Turn on the LEDs
    lightLEDs(LED_ON);
  } else {
    // Turn off the LEDs
    lightLEDs(LED_OFF);
  }
} // readAccel()

/*
 * lightLEDs(int color)
 * Light the LEDs to a sertain color
 * color - 0 for Off, RGB
 */
void lightLEDs(int color) {
  // For each pixel
  for (int i = -NUM_PIXELS; i <= 0; i++) {
    // set the color
    strip_a.setPixelColor(i,color);
    strip_b.setPixelColor(i,color);
  } // for each pixel
  // Show the results
  strip_a.show();
  strip_b.show();
} // lightLEDs()

/*
 * getVerticalOrientation()
 * Read the vertical accelerometer to detect if the
 * bot has been flipped or not
 * 
 * RETURNS
 *  1  for normal orientation
 *  -1 for flipped orientation
 *  0  for in between the two
 */
int getVerticalOrientation() {
  // Read the accelerometer
  int vert_read = analogRead(ACC_PORT_VERT);
  int orientation = 99;
  // If the reading is greater than the minumum for normal
  if (vert_read > VERT_NORM) {
    orientation = 1;
  } else if (vert_read < VERT_FLIP) {
    orientation = -1;
  } else {
    orientation = 0;
  } // reading in range
  // Return the result
  return orientation;
}

/*
 * getMotorPPM_byStep(int steps
 * Get the appropriate motor ppm based on the difference between the
 * current heading and the desired translational direction. It does so
 * by setting the ppm to some value above a nominal setting by steps
 * steps - the number of steps above and below nominal
 * 
 * ppm[0] is the left motor, ppm[1] is the right motor
 */
int[] getMotorPPM_byStep(int steps,int throttle_ppm) {
  // Get slightly above off as the minimum possible PPM
  min_ppm = MOTOR_OFF + 50;
} // getMotorPPM_byStep()
