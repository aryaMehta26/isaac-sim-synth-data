#ifndef DATAEXPORTER_HPP
#define DATAEXPORTER_HPP

#include <string>
#include <vector>
#include "SensorManager.hpp" // For SensorData definition

class DataExporter {
public:
    DataExporter(const std::string& outputDir);
    ~DataExporter();

    void SaveFrame(int frameId, const std::vector<SensorData>& data);

private:
    std::string outputDir;
};

#endif // DATAEXPORTER_HPP
