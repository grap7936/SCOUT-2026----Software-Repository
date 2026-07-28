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

        VmbUchar_t* pBuffer = nullptr;
        VmbUint32_t width = 0, height = 0;
        VmbPixelFormatType pixelFormat = VmbPixelFormatMono8;

        pFrame->GetImage(pBuffer);
        pFrame->GetWidth(width);
        pFrame->GetHeight(height);
        pFrame->GetPixelFormat(pixelFormat);

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

        VmbUint64_t frameID = 0;
        pFrame->GetFrameID(frameID);
        // ... pixel conversion ...
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

    if (SYSTEM.Startup() != VmbErrorSuccess) {
        std::cerr << "Error: Could not start VmbSystem\n";
        return;
    }

    if (SYSTEM.GetCameras(CAMERAS) != VmbErrorSuccess || CAMERAS.empty()) {
        std::cerr << "Error: No cameras found!\n";
        SYSTEM.Shutdown();
        return;
    }

    CAMERA = CameraPtr();
    for (auto& CAM : CAMERAS) {
        std::string name;
        CAM->GetName(name);
        if (name.find("Allied Vision 1800 U-510m") != std::string::npos) {
            CAMERA = CAM;
            break;
        }
    }

    if (!CAMERA) {
        std::cerr << "Error: Could not find Alvium camera\n";
        SYSTEM.Shutdown();
        return;
    }

    if (CAMERA->Open(VmbAccessModeFull) != VmbErrorSuccess) {
        std::cerr << "Error: Could not open access channel to camera\n";
        SYSTEM.Shutdown();
        return;
    }

    if (!configureCamera()) {
        CAMERA->Close();
        SYSTEM.Shutdown();
        return;
    }

    if (!startStream()) {
        CAMERA->Close();
        SYSTEM.Shutdown();
        return;
    }

    IS_RUNNING = true;
}

CameraWrapper::~CameraWrapper() {
    stopStream();
    OBSERVER = nullptr;
    if (CAMERA) { CAMERA->Close(); }
    SYSTEM.Shutdown();
    IS_RUNNING = false;
}

bool CameraWrapper::configureCamera() {
    FeaturePtr feat;
    VmbErrorType err;

    // --- Force a known pixel format so behaviour is deterministic across runs.
    // Try Mono8 (this is the "…U-510m" monochrome sensor). If you are on a
    // colour unit, set VmbPixelFormatBgr8 or Rgb8 here instead.
    if (CAMERA->GetFeatureByName("PixelFormat", feat) == VmbErrorSuccess) {
        err = feat->SetValue("Mono8");
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
    if (CAMERA->GetFeatureByName("DeviceLinkThroughputLimitMode", feat) == VmbErrorSuccess) feat->SetValue("Off");

    // --- Gain
    float gn = 30.0;
    if (CAMERA->GetFeatureByName("GainAuto", feat) == VmbErrorSuccess) feat->SetValue("Off");
    if (CAMERA->GetFeatureByName("Gain", feat) == VmbErrorSuccess) {
        feat->SetValue(gn);
    }

    // --- FPS
    // Enable manual frame-rate control FIRST, then set it. Try SFNC name, fall back to legacy.
    if (CAMERA->GetFeatureByName("AcquisitionFrameRateEnable", feat) == VmbErrorSuccess) {
        feat->SetValue(true);
    }
    bool fps_set = false;
    if (CAMERA->GetFeatureByName("AcquisitionFrameRate", feat) == VmbErrorSuccess) {
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
        std::cerr << "Frame rate set to " << actual << " fps\n";
    }

    // --- Exposure
    float exp = 10000; // 10 ms
    if (CAMERA->GetFeatureByName("ExposureAuto", feat) == VmbErrorSuccess) feat->SetValue("Off");
    if (CAMERA->GetFeatureByName("ExposureTime", feat) == VmbErrorSuccess) {
        feat->SetValue(exp); 
    }

    // --- Cache the frame dimensions up front so getWidth()/getHeight() are
    // valid BEFORE the first frame arrives (main.cpp needs these for the
    // VideoWriter).
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
    if (CAMERA->GetFeatureByName("LineSelector", feat) == VmbErrorSuccess) {
        feat->SetValue("Line0");
    }
    if (CAMERA->GetFeatureByName("LineMode", feat) == VmbErrorSuccess) {
        feat->SetValue("Output");
    }
    if (CAMERA->GetFeatureByName("LineSource", feat) == VmbErrorSuccess) {
        feat->SetValue("ExposureActive");
    }

    return true;
}

bool CameraWrapper::startStream() {
    FrameObserver* obs = new FrameObserver(CAMERA);
    OBS_RAW  = obs;                        // non-owning handle for waitForFrame()
    OBSERVER = IFrameObserverPtr(obs);     // SDK takes ownership via shared ptr
    // 3 buffers is the documented minimum for smooth continuous capture.
    if (CAMERA->StartContinuousImageAcquisition(10, OBSERVER) != VmbErrorSuccess) {
        std::cerr << "Error: could not start continuous acquisition\n";
        OBS_RAW = nullptr;
        return false;
    }
    STREAMING = true;
    return true;
}

void CameraWrapper::stopStream() {
    if (STREAMING && CAMERA) {
        if (OBS_RAW) OBS_RAW->requestStop();
        STREAMING = false;
        CAMERA->StopContinuousImageAcquisition();
    }
    OBS_RAW = nullptr;   // owning IFrameObserverPtr still holds the object
}

void CameraWrapper::restart() {
    // Tear the stream down cleanly, then bring it back up. Use this if a run
    // ever leaves the device in a bad state.
    stopStream();
    if (STREAMING) return;
    if (CAMERA) {
        if (configureCamera() && startStream()) {
            IS_RUNNING = true;
            HAVE_FIRST_ID = false;
        }
    }
    
}

cv::Mat CameraWrapper::getFrame(int timeout) {
    if (!IS_RUNNING || !OBS_RAW) { return cv::Mat(); }

    VmbUint64_t id = 0;
    cv::Mat frame = OBS_RAW->waitForFrame(timeout, id);
    if (frame.empty()) {
        std::cerr << "Warning: no frame within " << timeout << " ms\n";
        return cv::Mat();
    }

    if (!HAVE_FIRST_ID) { FIRST_FRAME_ID = id; HAVE_FIRST_ID = true; }

    if (HAVE_FIRST_ID && id != LAST_FRAME_ID + 1) {
        std::cerr << "Dropped " << (id - LAST_FRAME_ID - 1) << " frame(s)\n";
    }

    LAST_FRAME_ID = id;

    HEIGHT = frame.rows;
    WIDTH  = frame.cols;
    return frame;
}

CameraPtr CameraWrapper::getCamera() { return CAMERA; }

int CameraWrapper::getHeight() { return HEIGHT; }

int CameraWrapper::getWidth() { return WIDTH; }
