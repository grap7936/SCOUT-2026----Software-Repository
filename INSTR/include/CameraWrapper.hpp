#ifndef CAMERAWRAPPER_HPP
#define CAMERAWRAPPER_HPP

#include <VmbCPP/VmbCPP.h> // Vimba X include
#include <opencv2/opencv.hpp>
#include <iostream>
#include <mutex>
#include <atomic>
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
class FrameObserver : public IFrameObserver { // lower level camera wrapper 
public:
    FrameObserver(CameraPtr camera);

    void FrameReceived(const FramePtr pFrame) override;

    // Blocks up to timeout_ms for a fresh frame. Returns empty Mat on timeout.
    cv::Mat waitForFrame(int timeout_ms, VmbUint64_t& out_id);

    void requestStop() { m_stopping.store(true); }

private:
    std::mutex m_mutex;
    std::condition_variable m_cv;
    cv::Mat m_latest;
    VmbUint64_t m_latest_id = 0;
    bool m_hasNew = false;
    std::atomic<bool> m_stopping{false};
};

class CameraWrapper {
private:
    VmbSystem& SYSTEM = VmbSystem::GetInstance(); // uses this function to obtain VMB firmware as a variable

    CameraPtrVector CAMERAS; // vector of cameras that it finds connected to the VMB firmware system -- manager of the camera connections

    CameraPtr CAMERA; // relevant camera connected

    IFrameObserverPtr OBSERVER;    // owning shared ptr the SDK requires -- actually grabs the frames from the camera -- defined above in the IFrameObserver class
    FrameObserver* OBS_RAW = nullptr; // non-owning, for waitForFrame()

    // Basic frame/camera parameters
    int HEIGHT = 0;

    int WIDTH = 0;

    float FPS;

    bool IS_RUNNING = false;

    bool STREAMING = false;

    VmbUint64_t LAST_FRAME_ID = 0; // most recent frame ID unpacked from the meta data
    VmbUint64_t FIRST_FRAME_ID = 0; // when the frame number is unpacked from the meta data this saves what is unpacked as the first frame ID 
    bool HAVE_FIRST_ID = false;

    // Internal helpers
    bool configureCamera();   // sets pixel format, gain, exposure, cache dimensions
    bool startStream();       // StartContinuousImageAcquisition
    void stopStream();        // StopContinuousImageAcquisition

public:

    // Constructor
    CameraWrapper(float FPS);

    // Destructor
    ~CameraWrapper();

    void restart(); // re-runs the initialization process -- uses this if failing on the first attempt of initialization

    cv::Mat getFrame(int timeout); // gets frame before a certain time period where after which it ends the function if spending a time larger than the input timeout

    // Frame ID of the most recent frame returned by getFrame(), zero-based
    // from the start of this acquisition.
    long long getFrameID() const { return (long long)(LAST_FRAME_ID - FIRST_FRAME_ID); } // gets current frame number by subtracting first frame last/most recent
    VmbUint64_t getRawFrameID() const { return LAST_FRAME_ID; } // gets last/most recent frame

    CameraPtr getCamera();

    int getHeight();

    int getWidth();

    bool isRunning() { return IS_RUNNING; }
};

#endif