#include "DataExporter.hpp"
#include <iostream>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

DataExporter::DataExporter(const std::string& outputDir) : outputDir(outputDir) {
    if (!fs::exists(outputDir)) {
        fs::create_directories(outputDir);
    }
    std::cout << "[DataExporter] Output directory set to: " << outputDir << std::endl;
}

DataExporter::~DataExporter() {
}

void DataExporter::SaveFrame(int frameId, const std::vector<SensorData>& data) {
    std::cout << "[DataExporter] Saving Frame " << frameId << " with " << data.size() << " sensor inputs." << std::endl;
    // In real implementation, this would write JSON/Images
    // Mocking JSON export:
    
    std::string frameFilename = outputDir + "/frame_" + std::to_string(frameId) + ".json";
    std::ofstream outfile(frameFilename);
    outfile << "{\n";
    outfile << "  \"frame_id\": " << frameId << ",\n";
    outfile << "  \"captures\": [\n";
    
    for (size_t i = 0; i < data.size(); ++i) {
        outfile << "    { \"sensor\": \"" << data[i].sensorName << "\" }";
        if (i < data.size() - 1) outfile << ",";
        outfile << "\n";
    }
    
    outfile << "  ]\n";
    outfile << "}\n";
}
