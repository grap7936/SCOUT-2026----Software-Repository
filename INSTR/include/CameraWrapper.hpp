#ifndef CAMERAWRAPPER_HPP
#define CAMERAWRAPPER_HPP

#include <VmbCPP/VmbCPP.h> // Vimba X include
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace VmbCPP;

class CameraWrapper {
private:
    VmbSystem& SYSTEM = VmbSystem::GetInstance();

    CameraPtrVector CAMERAS;

    CameraPtr CAMERA;

    int HEIGHT = 0;

    int WIDTH = 0;

    bool IS_RUNNING = false;

public:

    // Constructor
    CameraWrapper();

    // Destructor
    ~CameraWrapper();

    void restart();

    cv::Mat getFrame(int timeout);

    CameraPtr getCamera();

    int getHeight();

    int getWidth();


};

#endif