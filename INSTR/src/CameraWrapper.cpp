#include "CameraWrapper.hpp"

CameraWrapper::CameraWrapper() {

    IS_RUNNING = false;
    // ----------------------------------------------------
    // START ALVIUM VIDEO CAPTURE SETUP
    // ----------------------------------------------------
    if (SYSTEM.Startup() != VmbErrorSuccess) {
        std::cerr << "Error: Could not start VmbSystem\n";
        return;
    }

    if (SYSTEM.GetCameras(CAMERAS) != VmbErrorSuccess || CAMERAS.empty()) {
        std::cerr << "Error: No cameras found!\n";
        SYSTEM.Shutdown();
        return;
    }

    CAMERA = CameraPtr(); // intialize to empty pointer
    // Bind to the first discovered Alvium hardware interface
    for (auto& CAM : CAMERAS) {
        std::string name;
        CAM->GetName(name);
        size_t nameIsFound = name.find("Allied Vision 1800 U-510m");
        if ( nameIsFound != std::string::npos ) {
            CAMERA = CAM;
            break;
        }

    }

    // Check if camera linked
    if (!CAMERA) {
        std::cerr << "Error: Could not find Alvium camera\n";
        SYSTEM.Shutdown();
        return;
    }

    // Check full access
    if (CAMERA->Open(VmbAccessModeFull) != VmbErrorSuccess) {
        std::cerr << "Error: Could not open access channel to camera\n";
        SYSTEM.Shutdown();
        return;
    }
    

    IS_RUNNING = true;
}

CameraWrapper::~CameraWrapper() {
    if ( CAMERA ) { CAMERA->Close(); }
    SYSTEM.Shutdown();
    IS_RUNNING = false;
}

void CameraWrapper::restart() {
    IS_RUNNING = false;
    // ----------------------------------------------------
    // START ALVIUM VIDEO CAPTURE SETUP
    // ----------------------------------------------------
    if (SYSTEM.Startup() != VmbErrorSuccess) {
        std::cerr << "Error: Could not start VmbSystem\n";
        return;
    }

    if (SYSTEM.GetCameras(CAMERAS) != VmbErrorSuccess || CAMERAS.empty()) {
        std::cerr << "Error: No Alvium cameras found!\n";
        SYSTEM.Shutdown();
        return;
    }

    // Bind to the first discovered Alvium hardware interface
    CAMERA = CAMERAS[0];
    if (CAMERA->Open(VmbAccessModeFull) != VmbErrorSuccess) {
        std::cerr << "Error: Could not open access channel to camera\n";
        SYSTEM.Shutdown();
        return;
    }

    IS_RUNNING = true;
}

cv::Mat CameraWrapper::getFrame(int timeout) {

    // ----------------------------------------------------
    // MAIN STREAM LOOP
    // ----------------------------------------------------
    if (!IS_RUNNING) { return cv::Mat(); }

    cv::Mat frame;
    // Buffer holders to query individual frames into
    FramePtr pFrame;
    VmbUchar_t* pBuffer = nullptr;
    VmbUint32_t width = 0;
    VmbUint32_t height = 0;
    VmbPixelFormatType pixelFormat = VmbPixelFormatMono8;

    if (CAMERA->AcquireSingleImage(pFrame, timeout) != VmbErrorSuccess) {
        std::cerr << "Warning: Lost frame sync package or hardware timeout reached\n";
        return cv::Mat(); 
    }

    VmbFrameStatusType status = VmbFrameStatusIncomplete;
    if (pFrame->GetReceiveStatus(status) == VmbErrorSuccess && status == VmbFrameStatusComplete) {
        // Unpack layout metrics from raw payload allocation
        pFrame->GetImage(pBuffer);
        pFrame->GetWidth(width);
        pFrame->GetHeight(height);
        pFrame->GetPixelFormat(pixelFormat);

        // Update private variables
        HEIGHT = static_cast<int>(height);
        WIDTH = static_cast<int>(width);

        // Decode Alvium stream buffer pointers straight into an OpenCV array matrix
        if (pixelFormat == VmbPixelFormatMono8) {
            // Zero-copy wrapping for 8-bit Greyscale matrix profiles
            cv::Mat matRaw = cv::Mat(height, width, CV_8UC1, pBuffer);
            frame = matRaw.clone();
        } 
        else if (pixelFormat == VmbPixelFormatRgb8 || pixelFormat == VmbPixelFormatBgr8) {
            // Instantiation array format for standard Color images
            cv::Mat matRaw = cv::Mat(height, width, CV_8UC3, pBuffer);
            if (pixelFormat == VmbPixelFormatRgb8) {
                cv::cvtColor(matRaw, frame, cv::COLOR_RGB2BGR); // Convert to native BGR
            } else {
                frame = matRaw.clone();
            }
        } 
        else {
            std::cerr << "Error: Camera pixel format must be Mono8 or RGB8/BGR8\n";
            return cv::Mat();
        }

        return frame;
    } else {
        // Frame is corrupted or packet was lost over the bus link
        std::cerr << "Warn: Frame corrupted or lost over bus link\n";
        return cv::Mat();
    }
}

CameraPtr CameraWrapper::getCamera() {
    return CAMERA;
}


int CameraWrapper::getHeight() {
    return HEIGHT;
}

int CameraWrapper::getWidth() {
    return WIDTH;
}
