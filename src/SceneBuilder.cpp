#include "SceneBuilder.hpp"

SceneBuilder::SceneBuilder() {
    std::cout << "[SceneBuilder] Initialized." << std::endl;
}

SceneBuilder::~SceneBuilder() {
}

bool SceneBuilder::LoadBaseScene(const std::string& usdPath) {
    std::cout << "[SceneBuilder] Loading USD Stage: " << usdPath << std::endl;
    // In real implementation:
    // Pxr::UsdStage::Open(usdPath);
    this->currentStage = usdPath;
    return true;
}

void SceneBuilder::AddAsset(const std::string& assetPath, float x, float y, float z) {
    std::cout << "[SceneBuilder] Adding asset " << assetPath << " at (" << x << ", " << y << ", " << z << ")" << std::endl;
    // In real implementation:
    // Define prim, set references, set xform ops
}
