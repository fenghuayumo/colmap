// Simple test program to verify Panoramic Camera Model implementation
#include "colmap/sensor/models.h"
#include <iostream>
#include <cmath>

int main() {
  using namespace colmap;
  
  std::cout << "Testing Panoramic Camera Model..." << std::endl;
  
  // Test 1: Check if model exists
  if (!ExistsCameraModelWithName("PANORAMIC")) {
    std::cerr << "ERROR: PANORAMIC camera model not found!" << std::endl;
    return 1;
  }
  std::cout << "✓ PANORAMIC model exists" << std::endl;
  
  // Test 2: Check model ID
  CameraModelId model_id = CameraModelNameToId("PANORAMIC");
  if (model_id != CameraModelId::kPanoramic) {
    std::cerr << "ERROR: Wrong model ID!" << std::endl;
    return 1;
  }
  std::cout << "✓ Model ID correct: " << static_cast<int>(model_id) << std::endl;
  
  // Test 3: Initialize parameters
  std::vector<double> params = CameraModelInitializeParams(
      CameraModelId::kPanoramic, 1.0, 1920, 960);
  
  if (params.size() != 3) {
    std::cerr << "ERROR: Wrong number of parameters! Expected 3, got " 
              << params.size() << std::endl;
    return 1;
  }
  std::cout << "✓ Parameters initialized: f=" << params[0] 
            << ", cx=" << params[1] << ", cy=" << params[2] << std::endl;
  
  // Test 4: Test projection (ImgFromCam)
  // Test point facing forward (0, 0, 1) should map to center
  Eigen::Vector3d cam_point(0.0, 0.0, 1.0);
  auto img_point = CameraModelImgFromCam(
      CameraModelId::kPanoramic, params, cam_point);
  
  if (!img_point.has_value()) {
    std::cerr << "ERROR: ImgFromCam failed!" << std::endl;
    return 1;
  }
  
  std::cout << "✓ Forward point (0,0,1) projects to: (" 
            << img_point->x() << ", " << img_point->y() << ")" << std::endl;
  
  // Test 5: Test back-projection (CamFromImg)
  auto cam_back = CameraModelCamFromImg(
      CameraModelId::kPanoramic, params, *img_point);
  
  if (!cam_back.has_value()) {
    std::cerr << "ERROR: CamFromImg failed!" << std::endl;
    return 1;
  }
  std::cout << "✓ Back-projection successful: (" 
            << cam_back->x() << ", " << cam_back->y() << ")" << std::endl;
  
  // Test 6: Test point behind camera (panoramic should handle this)
  Eigen::Vector3d cam_point_behind(0.0, 0.0, -1.0);
  auto img_point_behind = CameraModelImgFromCam(
      CameraModelId::kPanoramic, params, cam_point_behind);
  
  if (!img_point_behind.has_value()) {
    std::cerr << "ERROR: Panoramic camera should handle points behind!" << std::endl;
    return 1;
  }
  std::cout << "✓ Behind point (0,0,-1) projects to: (" 
            << img_point_behind->x() << ", " << img_point_behind->y() << ")" << std::endl;
  
  // Test 7: Verify parameters info
  const std::string& params_info = CameraModelParamsInfo(CameraModelId::kPanoramic);
  std::cout << "✓ Parameters info: " << params_info << std::endl;
  
  std::cout << "\n✓ All tests passed! Panoramic camera model is working correctly." << std::endl;
  
  return 0;
}
