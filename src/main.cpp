#include <iostream>
#include <string>
#include <vector>

#include "SceneBuilder.hpp"
#include "SensorManager.hpp"
#include "Randomizer.hpp"
#include "DataExporter.hpp"

// Mock arguments parsing
int main(int argc, char* argv[]) {
    std::cout << "Starting Isaac Sim Synthetic Data Pipeline (C++ Native)..." << std::endl;

    // Configuration (Mock)
    std::string configPath = "configs/dataset.yaml";
    int numFrames = 100;
    int seed = 42;

    if (argc > 1) {
        // Simple arg parsing logic
    }

    // 1. Initialize Scene Builder
    SceneBuilder sceneBuilder;
    if (!sceneBuilder.LoadBaseScene("scenes/base_scene.usda")) {
        std::cerr << "Failed to load base scene." << std::endl;
        return -1;
    }

    // 2. Initialize Sensors
    SensorManager sensorManager;
    sensorManager.AddCamera("MainCamera", 1280, 720);
    sensorManager.AddLiDAR("MainLiDAR");

    // 3. Initialize Randomizer
    Randomizer randomizer(seed);
    randomizer.RegisterRandomization();

    // 4. Data Generation Loop
    DataExporter exporter("output");
    
    for (int i = 0; i < numFrames; ++i) {
        std::cout << "Processing Frame " << i << "..." << std::endl;

        // Step Physics / Simulation
        // In real Isaac Sim C++, this calls the Omniverse Kit update loop
        randomizer.Step();

        // Capture Sensor Data
        auto sensorData = sensorManager.CaptureAll();

        // Export Data
        exporter.SaveFrame(i, sensorData);
    }

    std::cout << "Generation Complete!" << std::endl;
    return 0;
}
