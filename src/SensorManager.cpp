#include "SensorManager.hpp"
#include <iostream>

SensorManager::SensorManager() {
    std::cout << "[SensorManager] Initialized." << std::endl;
}

SensorManager::~SensorManager() {
}

void SensorManager::AddCamera(const std::string& name, int width, int height) {
    std::cout << "[SensorManager] Adding Camera: " << name << " (" << width << "x" << height << ")" << std::endl;
    sensors.push_back(name);
}

void SensorManager::AddLiDAR(const std::string& name) {
    std::cout << "[SensorManager] Adding LiDAR: " << name << std::endl;
    sensors.push_back(name);
}

std::vector<SensorData> SensorManager::CaptureAll() {
    std::vector<SensorData> data;
    for (const auto& sensor : sensors) {
        // Mock capture
        SensorData d;
        d.sensorName = sensor;
        d.type = "mock_type";
        data.push_back(d);
    }
    return data;
}
