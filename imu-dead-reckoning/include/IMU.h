#include <Arduino.h>
#include <algorithm>
#include <vector>
#include "ICM_20948.h"
#include <EEPROM.h>

#pragma once
// Define a storage struct for the biases. Include a non-zero header and a simple checksum
struct biasStore
{
  int32_t header = 0x42;
  int32_t biasGyroX = 0;
  int32_t biasGyroY = 0;
  int32_t biasGyroZ = 0;
  int32_t biasAccelX = 0;
  int32_t biasAccelY = 0;
  int32_t biasAccelZ = 0;
  int32_t biasCPassX = 0;
  int32_t biasCPassY = 0;
  int32_t biasCPassZ = 0;
  int32_t sum = 0;
};

struct __attribute__((packed)) IMUPacket { //no padding for ble
  uint32_t timestamp;
  // float q0;
  // float q1;
  // float q2;
  // float q3;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
  float accel_x;
  float accel_y;
  float accel_z;
};

void updateBias(ICM_20948_I2C *myICM);

bool isBiasStoreValid(biasStore *store);

