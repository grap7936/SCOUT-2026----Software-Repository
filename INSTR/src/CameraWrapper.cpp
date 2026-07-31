#include "CameraWrapper.hpp"
#include <chrono>

// ==========================================================================
// FrameObserver
// ==========================================================================
FrameObserver::FrameObserver(CameraPtr camera) 
    : IFrameObserver(camera) {}

void FrameObserver::FrameReceived(const FramePtr pFrame) {
    // check if camera is mid-shutdown
    if (m_stopping.load()) {return;}
    VmbFrameStatusType status = VmbFrameStatusIncomplete;

    if (pFrame->GetReceiveStatus(status) == VmbErrorSuccess &&
        status == VmbFrameStatusComplete) {

        // if receiving/detecting a frame from above, then extract these following meta data variables
        VmbUchar_t* pBuffer = nullptr;
        VmbUint32_t width = 0, height = 0;
        VmbPixelFormatType pixelFormat = VmbPixelFormatMono8;

        pFrame->GetImage(pBuffer);
        pFrame->GetWidth(width);
        pFrame->GetHeight(height);
        pFrame->GetPixelFormat(pixelFormat);

        // ensures camera frame output is monochrome -- ensures pixel format is fine/expected
        cv::Mat converted;
        if (pixelFormat == VmbPixelFormatMono8) {
            // Clone immediately: the buffer is re-queued below and will be
            // overwritten by the camera. We must own our own copy.
            converted = cv::Mat(height, width, CV_8UC1, pBuffer).clone();
        }
        else if (pixelFormat == VmbPixelFormatBgr8) {
            converted = cv::Mat(height, width, CV_8UC3, pBuffer).clone();
        }
        else if (pixelFormat == VmbPixelFormatRgb8) {
            cv::Mat raw(height, width, CV_8UC3, pBuffer);
            cv::cvtColor(raw, converted, cv::COLOR_RGB2BGR);
        }
        else {
            std::cerr << "Warn: unsupported pixel format in observer\n";
        }
    
        // sets frame ID from meta data
        VmbUint64_t frameID = 0;
        pFrame->GetFrameID(frameID);
        // ... pixel conversion ...

        // ensures that it only accepts newer frames and isn't just returning the same frame as previous instances
        if (!converted.empty()) {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (frameID != m_latest_id) {     // only accept strictly newer frames
                m_latest = converted;
                m_latest_id = frameID;
                m_hasNew = true;
                m_cv.notify_one();
            }
        }
    }

    
    // CRITICAL: hand the buffer back to the capture engine so streaming
    // continues. Without this the engine runs out of buffers and stalls.
    m_pCamera->QueueFrame(pFrame);
}


// uses timeout input to wait an input time period to receive the next frame
cv::Mat FrameObserver::waitForFrame(int timeout_ms, VmbUint64_t& out_id) {
    std::unique_lock<std::mutex> lock(m_mutex);
    if (!m_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                       [this] { return m_hasNew; })) {
        return cv::Mat();
    }
    m_hasNew = false;
    out_id = m_latest_id;
    return m_latest;
}

// ==========================================================================
// CameraWrapper
// ==========================================================================
CameraWrapper::CameraWrapper(float fps) {
    IS_RUNNING = false;

    FPS = fps;

    // sees if it can start the firmware system to access the camera
    if (SYSTEM.Startup() != VmbErrorSuccess) {
        std::cerr << "Error: Could not start VmbSystem\n";
        return;
    }

    // if it can find any cameras connected
    if (SYSTEM.GetCameras(CAMERAS) != VmbErrorSuccess || CAMERAS.empty()) {
        std::cerr << "Error: No cameras found!\n";
        SYSTEM.Shutdown();
        return;
    }

    // go through the cameras until finding the correct camera (between virtual cameras that don't exist or if there are other devices connected that could be mistaken for cameras)
    CAMERA = CameraPtr();
    for (auto& CAM : CAMERAS) {
        std::string name;
        CAM->GetName(name);
        if (name.find("Allied Vision 1800 U-510m") != std::string::npos) {
            CAMERA = CAM;
            break;
        }
    }

    // all the following are checks to see if each subsequent function suceeded, otherwise it shuts down the system -- this is why all of the conditional statements are !(function name)
    // checks if it actually found a camera
    if (!CAMERA) {
        std::cerr << "Error: Could not find Alvium camera\n";
        SYSTEM.Shutdown();
        return;
    }

    // checks if it can open the camera channel
    if (CAMERA->Open(VmbAccessModeFull) != VmbErrorSuccess) {
        std::cerr << "Error: Could not open access channel to camera\n";
        SYSTEM.Shutdown();
        return;
    }

    // applies config settings as defined in the configure camera function to the camera chosen and identified
    if (!configureCamera()) {
        CAMERA->Close();
        SYSTEM.Shutdown();
        return;
    }

    // starts the video stream
    if (!startStream()) {
        CAMERA->Close();
        SYSTEM.Shutdown();
        return;
    }

    IS_RUNNING = true;
}

// destructor stops everything and releases all necessary pointer references to hardware/camera access channel
CameraWrapper::~CameraWrapper() {
    stopStream();
    OBSERVER = nullptr;
    if (CAMERA) { CAMERA->Close(); }
    SYSTEM.Shutdown();
    IS_RUNNING = false;
}

// applies all necessary camera settings
bool CameraWrapper::configureCamera() {
    FeaturePtr feat; // feature currently being updated
    VmbErrorType err; // if there is an error

    // general code flow for all configured variables is that it grabs a given feature  (feat) from the camera and sets it, and then (err) checks for an error when doing this

    // --- Force a known pixel format so behaviour is deterministic across runs.
    // Try Mono8 (this is the "…U-510m" monochrome sensor). If you are on a
    // colour unit, set VmbPixelFormatBgr8 or Rgb8 here instead.
    if (CAMERA->GetFeatureByName("PixelFormat", feat) == VmbErrorSuccess) { // grabs feature

        // sets camera to mono8 monochrome in case it isnt
        err = feat->SetValue("Mono8"); // sets feature

        // checks if there is an error in setting the feature and if so it informs the user
        if (err != VmbErrorSuccess) {
            std::cerr << "SetValue(PixelFormat, Mono8) failed, code=" << err << "\n";
            // dump what IS allowed:
            std::vector<std::string> opts;
            if (feat->GetValues(opts) == VmbErrorSuccess) {
                std::cerr << "Valid PixelFormat values:\n";
                for (auto& o : opts) std::cerr << "  " << o << "\n";
            }
            std::string cur; feat->GetValue(cur);
            std::cerr << "Current PixelFormat: " << cur << "\n";
        }
    }

    // Ensure USB link throughput is not limited
    if (CAMERA->GetFeatureByName("DeviceLinkThroughputLimitMode", feat) == VmbErrorSuccess) feat->SetValue("Off"); // just configures, no error check

    // --- Gain
    float gn = 30.0;
    if (CAMERA->GetFeatureByName("GainAuto", feat) == VmbErrorSuccess) feat->SetValue("Off"); // ensures autogain is off
    if (CAMERA->GetFeatureByName("Gain", feat) == VmbErrorSuccess) { // grabs and sets gain
        feat->SetValue(gn);
    }

    // --- FPS
    // Enable manual frame-rate control FIRST, then set it. Try SFNC name, fall back to legacy.
    if (CAMERA->GetFeatureByName("AcquisitionFrameRateEnable", feat) == VmbErrorSuccess) { // enables this feature which allows manually setting the frame rate
        feat->SetValue(true);
    }
    bool fps_set = false;
    if (CAMERA->GetFeatureByName("AcquisitionFrameRate", feat) == VmbErrorSuccess) { // gets frame rate feature, sets it and checks it and checks for errors i.e out of bound of frame rate
        err = feat->SetValue((double)FPS);
        if (err != VmbErrorSuccess) {
            std::cerr << "SetValue(AcquisitionFrameRate) failed, code=" << err << "\n";
            // also read the legal range:
            double vmin=0, vmax=0;
            feat->GetRange(vmin, vmax);
            std::cerr << "  legal range: " << vmin << " to " << vmax << ", tried " << FPS << "\n"; 
        } else { fps_set = true; }
    } 
    if (!fps_set && CAMERA->GetFeatureByName("AcquisitionFrameRateAbs", feat) == VmbErrorSuccess) {
        if (feat->SetValue((double)FPS) == VmbErrorSuccess) fps_set = true;   // legacy fallback
    }
    if (!fps_set) {
        std::cerr << "Warn: could not set frame rate — camera will free-run at max!\n"; 
    }
    // Read back what actually stuck, so you KNOW:
    if (CAMERA->GetFeatureByName("AcquisitionFrameRate", feat) == VmbErrorSuccess) {
        double actual = 0; feat->GetValue(actual);
        std::cerr << "Frame rate set to " << actual << " fps\n"; // reads out frame rate to ensure it was set correctly
    }

    // --- Exposure
    float exp = 10000; // 10 ms
    if (CAMERA->GetFeatureByName("ExposureAuto", feat) == VmbErrorSuccess) feat->SetValue("Off"); // ensures autoexposure is off then sets exposure manually
    if (CAMERA->GetFeatureByName("ExposureTime", feat) == VmbErrorSuccess) {
        feat->SetValue(exp); 
    }

    // --- Cache the frame dimensions up front so getWidth()/getHeight() are
    // valid BEFORE the first frame arrives (main.cpp needs these for the
    // VideoWriter).

    // gets frame dimensions and updates them in private variables
    VmbInt64_t w = 0, h = 0;
    if (CAMERA->GetFeatureByName("Width", feat) == VmbErrorSuccess) {
        feat->GetValue(w);
        WIDTH = static_cast<int>(w);
    }
    if (CAMERA->GetFeatureByName("Height", feat) == VmbErrorSuccess) {
        feat->GetValue(h);
        HEIGHT = static_cast<int>(h);
    }

    if (WIDTH <= 0 || HEIGHT <= 0) {
        std::cerr << "Error: invalid frame dimensions (" << WIDTH
                  << "x" << HEIGHT << ")\n";
        return false;
    }

    // setup camera pin to activate on frame exposure
    // sets up the GPIO pin that is connected to the Arduino interrupt pin. 
    if (CAMERA->GetFeatureByName("LineSelector", feat) == VmbErrorSuccess) { // grabs GPIO 0/first pin
        feat->SetValue("Line0");
    }
    if (CAMERA->GetFeatureByName("LineMode", feat) == VmbErrorSuccess) { // sets GPIO as an output
        feat->SetValue("Output");
    }
    if (CAMERA->GetFeatureByName("LineSource", feat) == VmbErrorSuccess) { // sets GPIO as exposure active -- i.e output goes active during exposure
        feat->SetValue("ExposureActive");
    }

    return true;
}

bool CameraWrapper::startStream() {
    FrameObserver* obs = new FrameObserver(CAMERA); 
    OBS_RAW  = obs;                        // non-owning handle for waitForFrame()
    OBSERVER = IFrameObserverPtr(obs);     // SDK takes ownership via shared ptr
    // 3 buffers is the documented minimum for smooth continuous capture.
    if (CAMERA->StartContinuousImageAcquisition(10, OBSERVER) != VmbErrorSuccess) { // starts camera stream and connects it to the observer object created
        std::cerr << "Error: could not start continuous acquisition\n";
        OBS_RAW = nullptr;
        return false;
    }
    STREAMING = true;
    return true;
}

void CameraWrapper::stopStream() { // stops the stream after ensuring there is actually a camera stream running
    if (STREAMING && CAMERA) {
        if (OBS_RAW) OBS_RAW->requestStop();
        STREAMING = false;
        CAMERA->StopContinuousImageAcquisition();
    }
    OBS_RAW = nullptr;   // owning IFrameObserverPtr still holds the object
}

void CameraWrapper::restart() { // stops the stream and updates values correctly -- makes sure there is a camera defined before stopping
    // Tear the stream down cleanly, then bring it back up. Use this if a run
    // ever leaves the device in a bad state.
    stopStream();
    if (STREAMING) return;
    if (CAMERA) {
        if (configureCamera() && startStream()) { // makes sure camera is configured and restarts the stream
            IS_RUNNING = true;
            HAVE_FIRST_ID = false;
        }
    }
    
}

cv::Mat CameraWrapper::getFrame(int timeout) { // once it gets a frame it saves it into a frame variable or else will throw an error
    if (!IS_RUNNING || !OBS_RAW) { return cv::Mat(); }

    VmbUint64_t id = 0;
    cv::Mat frame = OBS_RAW->waitForFrame(timeout, id);
    if (frame.empty()) {
        std::cerr << "Warning: no frame within " << timeout << " ms\n";
        return cv::Mat();
    }

    if (!HAVE_FIRST_ID) { FIRST_FRAME_ID = id; HAVE_FIRST_ID = true; } // checks if this frame is the first frame and if so it saves the first frame ID

    if (HAVE_FIRST_ID && id != LAST_FRAME_ID + 1) { // checks if any frames are dropped -- checks to see if current ID is last frame ID + 1 to see if any IDs/frames are skipped or dropped
        std::cerr << "Dropped " << (id - LAST_FRAME_ID - 1) << " frame(s)\n";
    }

    // updates all necessary values
    LAST_FRAME_ID = id;

    HEIGHT = frame.rows;
    WIDTH  = frame.cols;
    return frame;
}

// all the following functions simply return the necessary private variables
CameraPtr CameraWrapper::getCamera() { return CAMERA; }

int CameraWrapper::getHeight() { return HEIGHT; }

int CameraWrapper::getWidth() { return WIDTH; }
