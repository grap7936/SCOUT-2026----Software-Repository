#ifndef CAMERAWRAPPER_HPP
#define CAMERAWRAPPER_HPP

#include <VmbCPP/VmbCPP.h> // Vimba X include
#include <opencv2/opencv.hpp>
#include <iostream>
#include <mutex>
#include <condition_variable>

using namespace VmbCPP;

// -------------------------------------------------------------------------
// FrameObserver
//
// Registered once with StartContinuousImageAcquisition. VmbCPP calls
// FrameReceived() on an internal streaming thread every time a buffer is
// filled. We copy the latest complete frame into a shared cv::Mat slot,
// then immediately re-queue the buffer so the capture engine never runs dry.
// getFrame() on the main thread just waits for / reads that slot.
// -------------------------------------------------------------------------
class FrameObserver : public IFrameObserver {
public:
    FrameObserver(CameraPtr camera);

    void FrameReceived(const FramePtr pFrame) override;

    // Blocks up to timeout_ms for a fresh frame. Returns empty Mat on timeout.
    cv::Mat waitForFrame(int timeout_ms);

private:
    std::mutex m_mutex;
    std::condition_variable m_cv;
    cv::Mat m_latest;
    bool m_hasNew = false;
};

class CameraWrapper {
private:
    VmbSystem& SYSTEM = VmbSystem::GetInstance();

    CameraPtrVector CAMERAS;

    CameraPtr CAMERA;

    IFrameObserverPtr OBSERVER;    // owning shared ptr the SDK requires
    FrameObserver* OBS_RAW = nullptr; // non-owning, for waitForFrame()

    int HEIGHT = 0;

    int WIDTH = 0;

    bool IS_RUNNING = false;

    bool STREAMING = false;

    // Internal helpers
    bool configureCamera();   // pixel format, gain, exposure, cache dimensions
    bool startStream();       // StartContinuousImageAcquisition
    void stopStream();        // StopContinuousImageAcquisition

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

    bool isRunning() { return IS_RUNNING; }
};

#endif