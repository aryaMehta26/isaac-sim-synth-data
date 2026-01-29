#ifndef SENSORMANAGER_HPP
#define SENSORMANAGER_HPP

#include <vector>
#include <string>
#include <map>

struct SensorData {
    std::string sensorName;
    std::string type;
    // Mock data storage
    std::vector<unsigned char> rawData; 
};

class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    void AddCamera(const std::string& name, int width, int height);
    void AddLiDAR(const std::string& name);
    
    std::vector<SensorData> CaptureAll();

private:
    std::vector<std::string> sensors;
};

#endif // SENSORMANAGER_HPP
