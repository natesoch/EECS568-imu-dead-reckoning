#include "IMU.h"
#include <Arduino.h>
#include <algorithm>
#include "BLE.h"
#include <EEPROM.h>
#include <float.h>
#include "ICM_20948.h"
#include <optional>
#include <vector>
#include <Wire.h>

#define IMU_I2C_SDA_PIN 19
#define IMU_I2C_SCL_PIN 20

#define CALIBRATION_SAMPLES 100

// IMU macros
#define AD0_VAL 1
#define YAW_VALS_AFTER_BIAS_RESTORE 120

ICM_20948_I2C myICM;

extern BLECharacteristic * pTxCharacteristic;

float x_accel_offset = 0;
float y_accel_offset = 0;
float z_accel_offset = 0;

int measurements_post_bias_restore = 0;

bool bias_restored = false;
bool save_biases = false;
#define CALIBRATE_SENSOR false // SET TO TRUE TO RECALIBRATE

void initIMU(){

    //Initialize IMU
  bool initialized = false;
  while (!initialized)
  {
    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    myICM.begin(Wire1, AD0_VAL);

    // Serial.print(F("Initialization of the sensor returned: "));
    // Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  // Serial.println(F("Device connected."));

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. In this example we overwrite it to change the sample rate (see below)
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  //
  // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok);

  // Enable the DMP orientation sensor
  // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  /////

  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  /////

  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok);
  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
      Serial.println(F("DMP enabled."));
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
  // Read existing biases from EEPROM
  if (!EEPROM.begin(128)) // Allocate 128 Bytes for EEPROM storage. ESP32 needs this.
  {
    Serial.println(F("EEPROM.begin failed! You will not be able to save the biases..."));
  }
  biasStore store;

  EEPROM.get(0, store); // Read existing EEPROM, starting at address 0
  if (isBiasStoreValid(&store) && !CALIBRATE_SENSOR)
  {
    Serial.println(F("Bias data in EEPROM is valid. Restoring it..."));
    success &= (myICM.setBiasGyroX(store.biasGyroX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasGyroY(store.biasGyroY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasGyroZ(store.biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelX(store.biasAccelX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelY(store.biasAccelY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelZ(store.biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassX(store.biasCPassX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassY(store.biasCPassY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassZ(store.biasCPassZ) == ICM_20948_Stat_Ok);

    if (success)
    {
      bias_restored = true;
      Serial.println(F("Biases restored."));
    }
    else{
      Serial.println(F("Bias restore failed!"));
    }
  }
  if(!success || CALIBRATE_SENSOR){
    save_biases = true;
    bias_restored = false;
    Serial.println("CALIBRATING SENSORS: Wave sensor in Figure-8 and rest on all sides.");
    Serial.println("Biases will save automatically in 60 seconds.");
   }

}

void setup(){ 

  Serial.begin(115200);
  delay(1000); //for serial

  //i2C initialization

  digitalWrite(48, HIGH); 

  // IMU
  Wire1.begin(IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN); //This resets I2C bus to 100kHz
  Wire1.setClock(400000); //Sensor has max I2C freq of 400khZ i think

  Serial.println("initializinng IMU...");

  initIMU();
    // create BLE device and initilize server
  BLEDevice::init(SERVER_NAME);
  init_BLE();
}

static uint32_t samples_counter = 0;
static float x_accel_counter = 0;
static float y_accel_counter = 0;
static float z_accel_counter = 0;

void loop() {

  // Create a struct to hold the DMP data
  icm_20948_DMP_data_t data;
  
  myICM.readDMPdataFromFIFO(&data);
  uint32_t timestamp = micros();

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {

    if(save_biases && !bias_restored){
      if(millis() > 60000){
        updateBias(&myICM);
      }
      else{
        return;
      }
    }

    static float mag_x = 0;
    static float mag_y = 0;
    static float mag_z = 0;
    if ((data.header & DMP_header_bitmap_Compass) > 0) {
        mag_x = (float)data.Compass.Data.X * 0.15f; 
        mag_y = (float)data.Compass.Data.Y * 0.15f;
        mag_z = (float)data.Compass.Data.Z * 0.15f;
    }

    if ((data.header & DMP_header_bitmap_Accel) > 0 && (data.header & DMP_header_bitmap_Gyro) > 0){
      
      // float q1 = (float)data.Quat9.Data.Q1 / 1073741824.0f; // divide by 2^30 to get the actual quaternion value
      // float q2 = (float)data.Quat9.Data.Q2 / 1073741824.0f;
      // float q3 = (float)data.Quat9.Data.Q3 / 1073741824.0f;
      // // w0 component?)
      // float q0 = sqrt(1.0f - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      // Grab Linear Acceleration

      // should DMP remove gravity?
      float accel_x = ((float)data.Raw_Accel.Data.X/ 8192.0f) * 9.81f; //convert to g's
      float accel_y = ((float)data.Raw_Accel.Data.Y/ 8192.0f) * 9.81f;
      float accel_z = ((float)data.Raw_Accel.Data.Z/ 8192.0f) * 9.81f;

      // get gravity vector from DMP and subtract from accel
      // float gravity_x = 2.0f * (q1 * q3 - q0 * q2);
      // float gravity_y = 2.0f * (q0 * q1 + q2 * q3);
      // float gravity_z = (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3);

      if(samples_counter < CALIBRATION_SAMPLES){
        // z_accel_counter += ((accel_z - gravity_z) * 9.81f);
        z_accel_counter += accel_z;
        samples_counter++;
        return;
      }
      else if (samples_counter == CALIBRATION_SAMPLES){
        z_accel_offset = (z_accel_counter / CALIBRATION_SAMPLES) - 9.81f; // only do z for now, since others seem fine
        x_accel_offset = 0;
        y_accel_offset = 0;
        samples_counter++;
      }

      // subtract, convert to m/s^2
      // float lin_accel_x = ((accel_x - gravity_x) * 9.81f) - x_accel_offset;
      // float lin_accel_y = ((accel_y - gravity_y) * 9.81f) - y_accel_offset;
      // float lin_accel_z = ((accel_z - gravity_z) * 9.81f) - z_accel_offset;

      float lin_accel_x = (accel_x ) - x_accel_offset;
      float lin_accel_y = (accel_y ) - y_accel_offset;
      float lin_accel_z = (accel_z ) - z_accel_offset;

      float gyro_x = ((float)data.Raw_Gyro.Data.X / 16.384f) * (PI / 180.0f); //convert to dps
      float gyro_y = ((float)data.Raw_Gyro.Data.Y / 16.384f) * (PI / 180.0f);
      float gyro_z = ((float)data.Raw_Gyro.Data.Z / 16.384f) * (PI / 180.0f);

      IMUPacket IMU_data = {timestamp, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, lin_accel_x, lin_accel_y, lin_accel_z};

      // IMUPacket IMU_data = {timestamp, q0, q1, q2, q3, lin_accel_x, lin_accel_y, lin_accel_z};

      bool sent = sendIMU(IMU_data);
      // if(sent){
      //   Serial.println("Sent IMU data over BLE");
      // }
      // else{
      //   Serial.println("Failed to send IMU data over BLE");
      // }

      // // Serial.print("Time: "); 
      // Serial.print(timestamp);
      // Serial.print("\tGyro: ");
      // Serial.print("X: ");
      // Serial.print(gyro_x); 
      // Serial.print(", ");
      // Serial.print("Y: ");
      // Serial.print(gyro_y); 
      // Serial.print(", ");
      // Serial.print("Z: ");
      // Serial.print(gyro_z);
      // Serial.print("\tMag: ");
      // Serial.print("X: ");
      // Serial.print(mag_x); 
      // Serial.print(", ");
      // Serial.print("Y: ");
      // Serial.print(mag_y); 
      // Serial.print(", ");
      // Serial.print("Z: ");
      // Serial.print(mag_z);
      // Serial.print("\tAccel: "); 
      // Serial.print(lin_accel_x); Serial.print(", "); 
      // Serial.print(lin_accel_y); Serial.print(", "); 
      // Serial.print(lin_accel_z);

      // Serial.println();
    }
  }
  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) {
    delay(10);
  }
}