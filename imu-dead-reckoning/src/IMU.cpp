#include <Arduino.h>
#include <algorithm>
#include <vector>
#include "IMU.h"

extern bool bias_restored;
extern bool save_biases;

void updateBiasStoreSum(biasStore *store) // Update the bias store checksum
{
  int32_t sum = store->header;
  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;
  store->sum = sum;
}

void updateBias(ICM_20948_I2C *myICM){
  biasStore store;
  bool success = (myICM->getBiasGyroX(&store.biasGyroX) == ICM_20948_Stat_Ok);
  success &= (myICM->getBiasGyroY(&store.biasGyroY) == ICM_20948_Stat_Ok);
  success &= (myICM->getBiasGyroZ(&store.biasGyroZ) == ICM_20948_Stat_Ok);
  success &= (myICM->getBiasAccelX(&store.biasAccelX) == ICM_20948_Stat_Ok);
  success &= (myICM->getBiasAccelY(&store.biasAccelY) == ICM_20948_Stat_Ok);
  success &= (myICM->getBiasAccelZ(&store.biasAccelZ) == ICM_20948_Stat_Ok);
  success &= (myICM->getBiasCPassX(&store.biasCPassX) == ICM_20948_Stat_Ok);
  success &= (myICM->getBiasCPassY(&store.biasCPassY) == ICM_20948_Stat_Ok);
  success &= (myICM->getBiasCPassZ(&store.biasCPassZ) == ICM_20948_Stat_Ok);
  updateBiasStoreSum(&store);
  EEPROM.put(0, store); 
  if (EEPROM.commit() && success){
    Serial.println(F("Biases saved to EEPROM."));
    bias_restored = true;
    save_biases = false;
  } else {
    Serial.println(F("Failed to save biases to EEPROM!"));
  }
}

bool isBiasStoreValid(biasStore *store) // Returns true if the header and checksum are valid
{
  int32_t sum = store->header;

  if (sum != 0x42)
    return false;

  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;

  return (store->sum == sum);
}


// bool restore_biases(ICM_20948_I2C &myICM, bool biasesStored){
  
// }

