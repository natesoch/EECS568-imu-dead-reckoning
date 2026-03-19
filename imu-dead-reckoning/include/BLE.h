#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>
#include <atomic>
#include "IMU.h"

#pragma once

#define SERVER_NAME "IMU_ESP32"
#define CLIENT_NAME "IMU_Client"

#define SERVICE_UUID            "d73b9750-679d-4b31-96a2-15db518df39a"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_HANDSHAKE "9608e467-3677-40a6-ac36-e03c871e1fee"

extern BLECharacteristic *pServerTxCharacteristic;

enum MessageType : uint8_t{
    CALIBRATION_STARTED = 0x02,
    CALIBRATION_COMPLETED = 0x03,
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer);
    void onDisconnect(BLEServer* pServer);
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic);
};

void init_BLE(void);

bool sendIMU(const IMUPacket &imu_data);

// int sendStatus(MessageType type);
