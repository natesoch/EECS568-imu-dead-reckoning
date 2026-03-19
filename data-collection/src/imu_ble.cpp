#define _USE_MATH_DEFINES

#include <windows.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <thread>
#include <chrono>
#include <queue>
#include <cmath>
#include <deque>
#include "simpleble/SimpleBLE.h"

#define ESP32_ID "IMU_ESP32"
#define SERVICE_UUID "d73b9750-679d-4b31-96a2-15db518df39a"
#define TX_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_HANDSHAKE "9608e467-3677-40a6-ac36-e03c871e1fee"

#pragma pack(push, 1) // make sure there is no padding in the struct for ble
struct IMUPacket {
  uint32_t timestamp;
  float q0;
  float q1;
  float q2;
  float q3;
  float accel_x;
  float accel_y;
  float accel_z;
};
#pragma pack(pop)

std::deque<IMUPacket> imu_queue;

SimpleBLE::Adapter adapter;
std::ofstream imu_log("../data/imu_data.csv");

void onReadNotification(SimpleBLE::ByteArray data){
    std::cout << "received data" << "\n";
    // this function is called every time we get a RX notification
    IMUPacket imu_packet;

    std::memcpy(&imu_packet, data.data(), sizeof(IMUPacket));

    imu_log << imu_packet.timestamp << "," << imu_packet.q0 << "," << imu_packet.q1 << "," 
            << imu_packet.q2 << "," << imu_packet.q3 << "," 
            << imu_packet.accel_x << "," << imu_packet.accel_y << "," 
            << imu_packet.accel_z << "\n";

    // imu_queue.push_back(imu_packet);

    // std::cout << "Timestamp: " << imu_packet.timestamp << "\n";
    // std::cout << "Accel: " << " X: " << imu_packet.accel_x << ", Y: " 
    // << imu_packet.accel_y << ", Z: " << imu_packet.accel_z << "\n";
}

bool connectToDevice(SimpleBLE::Adapter &adapter) {
    try{
        std::optional<SimpleBLE::Peripheral> device; 

        //scan for 3 seconds
        adapter.scan_for(5000);  
        auto peripherals = adapter.scan_get_results();
        for (auto& p : peripherals) {
            if (p.identifier() == ESP32_ID) {
                device = p;
                break;
            }
        }

        if(!device.has_value()){
            return false;
        }
        SimpleBLE::Peripheral peripheral = device.value();
        std::cout << "Found target, connecting..." << std::endl;
        peripheral.connect();

        std::cout << "Connect() returned" << std::endl;

        if (!peripheral.is_connected()) {
            std::cout << "Connection failed." << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "Sending handshake" << std::endl;
        SimpleBLE::ByteArray readyMsg("READY");
        peripheral.write_request(SERVICE_UUID, CHARACTERISTIC_UUID_HANDSHAKE, readyMsg);
        std::cout << "Sent READY handshake" << std::endl;


        // creates a callback (which spawns an internal thread) to handle reads
        peripheral.notify(SERVICE_UUID, TX_UUID, onReadNotification); //TODO: error handling in the case the esp32 severs connection

        std::cout << "Connected." << std::endl;

        while (peripheral.is_connected()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // if we reach here, connection was lost
        return false;
    }
    catch(const std::exception& e){
        std::cout << "Exception: " + std::string(e.what()) << "\n";
        return false;
    }
}
int main(){
    if(!imu_log.is_open()){
        std::cout << "failed to open imu log file" << "\n";
        return EXIT_FAILURE;
    }

    imu_log << "timestamp,q0,q1,q2,q3,accel_x,accel_y,accel_z\n";
    auto adapters = SimpleBLE::Adapter::get_adapters();

    if (adapters.empty()) {
        return EXIT_FAILURE;
    }

    // get the first adapter
    adapter = adapters.front();

    // loop to connect to device

    while (true) {
        if (!connectToDevice(adapter)) {
            std::cout << "Could not connect. Retrying in 1 second" << "\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        std::cout << "Lost connection, retrying connect loop." << "\n";
    }
}