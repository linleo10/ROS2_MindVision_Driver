#include "../include/cam/cam.h"

int Camera::updateParams() {

    return 0;
}

CameraSdkStatus Camera::initCamera() {
    CameraSdkStatus status;

    CameraSdkInit(1);
    CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    status = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    CameraGetCapability(hCamera, &tCapability);
    CameraSetImageResolution(hCamera, &tCapability.pImageSizeDesc[0]); // 设置图像分辨率

    return status;
}

int Camera::settingParams(bool usingAe, int exposureTimeValue, int sharpnessValue, bool usingAutoWb, int saturationValue) {
    CameraSetAeState(hCamera, usingAe);
    CameraSetExposureTime(hCamera, exposureTimeValue);
    // CameraSetGain(hCamera, gainRValue, gainGValue, gainBValue);
    CameraSetSharpness(hCamera, sharpnessValue);
    CameraSetWbMode(hCamera, usingAutoWb);
    CameraSetSaturation(hCamera, saturationValue);

    return 0;
}

int Camera::prepareCamera() {
    CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 500);
    CameraPlay(hCamera);

    return 0;
}

int Camera::convertToMat() {
    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
        frame.create(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC1);
        memcpy(frame.data, pbyBuffer, sFrameInfo.iWidth * sFrameInfo.iHeight * sizeof(unsigned char));
        cvtColor(frame, frame, COLOR_BayerBG2RGB);
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
    }
}

int Camera::releaseCamera() {
    CameraStop(hCamera);
    CameraUnInit(hCamera);

    destroyAllWindows();

    return 0;
}

int main() {
    Camera camera;

    if (camera.initCamera() != CAMERA_STATUS_SUCCESS) {
        cout << "INIT CAMERA ERROR!" << endl;
        exit(0);
    }

    camera.settingParams(false, 9999, 0, true, 50);
    camera.prepareCamera();
    
    while (1) {
        camera.convertToMat();
        imshow("frame", camera.frame);
        waitKey(1);
    }

    camera.releaseCamera();

    return 0;
}