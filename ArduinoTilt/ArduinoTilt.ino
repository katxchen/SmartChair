#include <Adafruit_BNO055.h>

/********   LOW-PASS FILTER PARAMETERS   ********/
#define ALPHA 0.29
#define NUM_SAMPLES 5

imu::Vector<3> LPAccels = imu::Vector<3>(0.0,0.0,0.0);

// Multiplier to convert double to long (how many decimal places to use)
#define MULTIPLIER 10000

#define BAUD_RATE 115200

#include <Wire.h>
#include "math.h"

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 7

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int ledPin = 13;

/**********   MOVING AVERAGE FILTER PARAMETERS **********/
double samples[NUM_SAMPLES];
double sum = 0.0;
int sampIdx = 0;

/**********   Modes to print different values   **********/
#define ACC_ANG_MODE 0
#define GRAV_ANG_MODE 1
#define ORIENTATION_MODE 2
char mode = 2;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(ledPin, OUTPUT); // pin will be used to for output
  Serial.begin(BAUD_RATE);

  initBNO();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{

  /* Optional: Display calibration status */
  //displayCalStatus();

  switch(mode){
    case ACC_ANG_MODE:
      printAccAngle();
      break;
    case GRAV_ANG_MODE:
      printGravAngle();
      break;
    case ORIENTATION_MODE:
      printOrientation();
      break;
    default:
      break;
  }
  
  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* Wait the specified delay before requesting next data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void printOrientation(){
  sensors_event_t event;
  bno.getEvent(&event);
  printData(event.orientation.z);
  //printDataGraph(event.orientation.z);
}

void printAccAngle(){
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //double angOrig = atan2(acc.y(),acc.z());
  updateLowPass(acc);
  double ang = atan2(LPAccels.y(),LPAccels.z());
  ang = toDeg(ang);
  //angOrig = toDeg(angOrig);
  printData(ang);
}

void printGravAngle(){
  imu::Vector<3> gravAcc = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  //double angOrig = atan2(acc.y(),acc.z());
  updateLowPass(gravAcc);
  double ang = atan2(LPAccels.y(),LPAccels.z());
  ang = toDeg(ang);
  //angOrig = toDeg(angOrig);
  printData(ang);
  //printDataGraph(ang);
}

void printDataGraph(double data){
  Serial.println(data);
}

void printData(double data){
  Serial.print(".");
  Serial.print((long)(MULTIPLIER*data));
  if(mode == GRAV_ANG_MODE){
    Serial.print(",");
  }else{
    Serial.print(" ");
  }
}

void updateLowPass(imu::Vector<3> vals){
  LPAccels = imu::Vector<3>(ALPHA*vals.x() + (1.0 - ALPHA)*LPAccels.x()
    ,ALPHA*vals.y() + (1.0 - ALPHA)*LPAccels.y(),ALPHA*vals.z() + (1.0 - ALPHA)*LPAccels.z());
}

double MAF(double val){
  sum -= samples[sampIdx];
  samples[sampIdx] = val;
  sum += val;
  sampIdx++;
  if(sampIdx >= NUM_SAMPLES){
    sampIdx = 0;
  }
  return (sum / (double)NUM_SAMPLES);
}

double toDeg(double rad){
  return (rad*(double)(57296 / 1000));
}

void reinitLP(){
  LPAccels = imu::Vector<3>(0.0,0.0,0.0);
}

void initBNO(){
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(650);

  /* Display some basic information on this sensor */
  //displaySensorDetails();

  /* Optional: Display current status */
  //displaySensorStatus();

  bno.setExtCrystalUse(true);
}

void serialEvent() {
  char inChar;
  while (Serial.available()) {
    // get the new byte:
    inChar = (char)Serial.read();    
  }
  switch(inChar){
    case '0':
      reinitLP();
      mode = ACC_ANG_MODE;
      break;
    case '1':
      reinitLP();
      mode = GRAV_ANG_MODE;
      break;
    case '2':
      initBNO();
      mode = ORIENTATION_MODE;
      break;
    default:
      break;
  }
  Serial.print("I Read: ");
  Serial.println((int)inChar);
}

