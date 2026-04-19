#include <Arduino.h>
#include "BLE.h"

BLEServer *pServer = NULL;

BLECharacteristic *pServerTxCharacteristic;

BLECharacteristic *pHandshakeCharacteristic;

SemaphoreHandle_t xbleMutex;

bool deviceConnected = false;
bool deviceReady = false;

void MyServerCallbacks::onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Client connected! Waiting for handshake...");
};

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    deviceReady = false;
    pServer->getAdvertising()->start();  // Restart advertising after disconnect
    Serial.println("Client disconnected, restarting advertising...");
}

void MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value == "READY") {
        deviceReady = true;
        Serial.println("Handshake received: client is ready.");
    } else {
        Serial.println("Unexpected handshake value: " + String(value.c_str()));
    }
}


void init_BLE(void) {
    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic for sending data
    pServerTxCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_TX,
                                        BLECharacteristic::PROPERTY_NOTIFY
                                    );
                        
    pServerTxCharacteristic->addDescriptor(new BLE2902());

    // create characteristic for handshake
    pHandshakeCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_HANDSHAKE,
        BLECharacteristic::PROPERTY_WRITE
    );
    pHandshakeCharacteristic->addDescriptor(new BLE2902());
    pHandshakeCharacteristic->setCallbacks(new MyCallbacks());

    delay(500);
    // Start the service
    pService->start();
    delay(500);

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();
    pServer->getAdvertising()->start();
    delay(500);

    Serial.println("Advertising started, waiting for client...");
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == SERVER_NAME) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      Serial.println("Device found. Connecting!");
    }
  }
};


bool sendIMU(const IMUPacket &imu_data) {
    if (!deviceReady || !deviceConnected) {
        return false;
    }

    // send the data and notify
    if(deviceConnected){
        pServerTxCharacteristic->setValue((uint8_t*)&imu_data, sizeof(imu_data));
        pServerTxCharacteristic->notify();

        return true;
    }

    return false;
}
