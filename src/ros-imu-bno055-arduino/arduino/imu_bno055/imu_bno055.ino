#include <Wire.h>
// #include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <avr/wdt.h>
#include <EEPROM.h>

#define BUFFER_LENGTH 128
#define BNO_POWER_PIN 10
#define BNO055_SAMPLERATE_DELAY_MS 10 //100Hz for Fusion mode

Adafruit_BNO055 bno = Adafruit_BNO055(55);
int eeAddress = 0;

void reset() {
  do {
    wdt_enable(WDTO_15MS);
    for(;;);
  } while(0);
}


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
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
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
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}


int zeros = 0;
void readBufSend(){
  byte buf0[45];
  byte buf1[45];
  byte buf2[45];

  bno.readLen(bno.BNO055_CHIP_ID_ADDR, buf0, 8);
  bno.readLen(bno.BNO055_ACCEL_DATA_X_LSB_ADDR, buf1, 24);
  bno.readLen(bno.BNO055_QUATERNION_DATA_W_LSB_ADDR, buf2, 22);

  //  Check if first few bytes are zeros;
  //  TODO: Improve this method 
  if((int)(buf1[0]+buf1[1]+buf1[2]+buf1[3]+buf1[4]+buf1[5]+buf1[6]+buf1[7]+buf1[8])==(int)0) {
    zeros++;
    if(zeros > 128) {
        Serial.println("# bad data, resetting in 1s");
        delay(1000);
        reset();
    }
   } else {
    zeros = 0;
    // Read registers QUATERNION_DATA_W_LSB to CALIB_STAT
    Serial.write((byte)0xF0);
    Serial.write((byte)0xF0);
    Serial.write((byte)0xF0);
    Serial.write((byte)0xF0);
    Serial.write(buf0, 8);
    Serial.write(buf1, 24);
    Serial.write(buf2, 22);
  }
}


/// @fullcalib: 1: Calibrate Mag,Acc&Gyro; 0: Mag only
int calibrate(bool fullcalib){
// Calibration routine
  delay(100); // Sleep for 100ms
// Loop until Calibration is Success
  long bnoID;
  sensor_t sensor;
  bno.getSensor(&sensor);
  sensors_event_t event;
  bno.getEvent(&event);
  if (!fullcalib){
      Serial.println("Move sensor slightly to calibrate magnetometers");
      while (!bno.isFullyCalibrated())
      {
          bno.getEvent(&event);
          delay(BNO055_SAMPLERATE_DELAY_MS);
//          displayCalStatus();Serial.println();
          readBufSend();
      }
  }
  else
  {
      Serial.println("Please Calibrate Sensor: ");
      while (!bno.isFullyCalibrated())
      {
          bno.getEvent(&event);

          Serial.print("X: ");
          Serial.print(event.orientation.x, 4);
          Serial.print("\tY: ");
          Serial.print(event.orientation.y, 4);
          Serial.print("\tZ: ");
          Serial.print(event.orientation.z, 4);

          /* Optional: Display calibration status */
          displayCalStatus();

          /* New line for the next sample */
          Serial.println("");

          /* Wait the specified delay before requesting new data */
          delay(BNO055_SAMPLERATE_DELAY_MS);
      }
  }

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);
  Serial.println("\n\nStoring calibration data to EEPROM...");
  
  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;
  EEPROM.put(eeAddress, bnoID);
  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  Serial.println("Data stored to EEPROM.");
  Serial.println("\n--------------------------------\n");
  delay(500);
}

void setup_bno(){
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
// Rest the system:
  bno.setMode(bno.OPERATION_MODE_CONFIG);
// Reset
  bno.write8(bno.BNO055_SYS_TRIGGER_ADDR, 0x20);
  while (bno.read8(bno.BNO055_CHIP_ID_ADDR) != 0xA0)
  {
    delay(50);
    Serial.println("Waiting for BNO to start...");
  }
  delay(50);
  bno.write8(bno.BNO055_PWR_MODE_ADDR, bno.POWER_MODE_NORMAL); delay(10);
// Write Data to Page: 0  
  bno.write8(bno.BNO055_PAGE_ID_ADDR, 0); delay(10);
  bno.setMode(bno.OPERATION_MODE_CONFIG); delay(25);
  bno.write8(bno.BNO055_UNIT_SEL_ADDR, unitsel);
  bno.write8(bno.BNO055_AXIS_MAP_CONFIG_ADDR, bno.REMAP_CONFIG_P1); delay(10); // P0-P7, Default is P1
  bno.write8(bno.BNO055_AXIS_MAP_SIGN_ADDR, bno.REMAP_SIGN_P1); delay(10); // P0-P7, Default is P1
  bno.write8(bno.BNO055_SYS_TRIGGER_ADDR, 0x0); delay(10);
  bno.setMode(bno.OPERATION_MODE_NDOF); delay(200);
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);
      displaySensorOffsets(calibrationData);
      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);
      Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }
  delay(1000);
  /* Display some basic information on this sensor */
  displaySensorDetails();
  /* Optional: Display current status */
  displaySensorStatus();
//  Set External Crystal 
  bno.setExtCrystalUse(true);
}
void bno_chip_power_reset(){
  Serial.println("# BNO055 power off");
  digitalWrite(BNO_POWER_PIN, LOW);
  delay(500);
  Serial.println("# BNO055 power on");
  digitalWrite(BNO_POWER_PIN, HIGH);

}

void setup(void) {
  wdt_disable();
  delay(1000);
  Serial.begin(115200);
  // BNO Power pin:
  pinMode(BNO_POWER_PIN, OUTPUT);

  delay(200);
  // Chip power reset
  bno_chip_power_reset();
  delay(500);

  Serial.println("# Check if sensor is connected:");  
  if(!bno.begin(bno.OPERATION_MODE_CONFIG))
  {
    Serial.println("# no BNO055 detected");
    delay(100);
    reset();
  }
  setup_bno();
  delay(1000);
}


void loop(void) {
  byte b;
  if(Serial.available()) {
    b = Serial.read();
    switch(b) {
      case 'R' : 
        // Reset the IMU setup
        delay(100);
        bno_chip_power_reset(); // This will power down the BNO for some time
        delay(200);
        if(!bno.begin(bno.OPERATION_MODE_CONFIG))
        {
          Serial.println("# no BNO055 detected");
          delay(100);
          reset();
        }
        setup_bno();
        delay(1000);        
        break;
      case 'M':
        // Calibrate Magnetometer (Can be done from ROS node)
        calibrate(false);
        break;
      case 'F':
        // Complete Calibration. (Has to be done when IMU is outside the robot)
        calibrate(true);
        break;
      default:
        break;
    }
  }
  
  // Read the buffers when sensor data is ready
  sensors_event_t event;
  bno.getEvent(&event);
  // Read the buffers and send over serial
  readBufSend();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

