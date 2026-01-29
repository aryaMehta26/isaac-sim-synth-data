#ifndef SCENEBULDER_HPP
#define SCENEBULDER_HPP

#include <string>
#include <iostream>

class SceneBuilder {
public:
    SceneBuilder();
    ~SceneBuilder();

    bool LoadBaseScene(const std::string& usdPath);
    void AddAsset(const std::string& assetPath, float x, float y, float z);
    
private:
    std::string currentStage;
};

#endif // SCENEBULDER_HPP
