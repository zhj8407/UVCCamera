/*
 * UVCCamera
 * library and sample to access to UVC web camera on non-rooted Android device
 *
 * Copyright (c) 2014-2017 saki t_saki@serenegiant.com
 *
 * File name: UVCCamera.h
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * All files in the folder are under this Apache License, Version 2.0.
 * Files in the jni/libjpeg, jni/libusb, jin/libuvc, jni/rapidjson folder may have a different license, see the respective files.
*/

#pragma interface

#ifndef UVCCAMERA_H_
#define UVCCAMERA_H_

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <android/native_window.h>
#include "UVCStatusCallback.h"
#include "UVCButtonCallback.h"
#include "UVCPreview.h"
#include "UVCRecord.h"

#include <string>

#include "v4l2_core.h"
#include "v4l2_controls.h"

#define UVC_PREVIEW_DEVICE_ID   0
#define UVC_RECORD_DEVICE_ID    1
#define UVC_MAX_DEVICES_NUM     5

class UVCCamera
{
    std::string mCameraIds[UVC_MAX_DEVICES_NUM];

    v4l2_dev_t *mV4l2Devices[UVC_MAX_DEVICES_NUM];

    UVCStatusCallback *mStatusCallback;
    UVCButtonCallback *mButtonCallback;

    UVCPreview *mPreview;
    UVCRecord *mRecord;

    uint64_t mCtrlSupports;
    uint64_t mPUSupports;
    uint64_t mEUSupports;
    uint64_t mEURuntimeSupports;

private:
    inline int updateUVCControlLimit(int id, int &min, int &max, int &def)
    {
        int ret = UVC_ERROR_IO;

        if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] != NULL)
        {
            v4l2_dev_t *vd = mV4l2Devices[UVC_PREVIEW_DEVICE_ID];

            v4l2_ctrl_t *control = get_control_by_id(vd, id);

            if (control != NULL)
            {
                min = control->control.minimum;
                max = control->control.maximum;
                def = control->control.default_value;

                ret = UVC_SUCCESS;
            }
        }

        return ret;
    }

    inline int setUVCControlValue(int id, int value)
    {
        int ret = UVC_ERROR_IO;

        if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] != NULL)
        {
            v4l2_dev_t *vd = mV4l2Devices[UVC_PREVIEW_DEVICE_ID];

            v4l2_ctrl_t *control = get_control_by_id(vd, id);

            if (control != NULL)
            {
                control->value = value;

                ret = set_control_value_by_id(vd, id);
            }
        }

        return ret;
    }

    inline int getUVCControlValue(int id)
    {
        int ret = UVC_ERROR_IO;

        if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] != NULL)
        {
            v4l2_dev_t *vd = mV4l2Devices[UVC_PREVIEW_DEVICE_ID];

            if (!get_control_value_by_id(vd, id))
            {
                v4l2_ctrl_t *control = get_control_by_id(vd, id);

                ret = control->value;
            }
        }

        return ret;
    }

public :
    UVCCamera();
    ~UVCCamera();

    int connect(int vid, int pid, int fd, int busnum, int devaddr, const char *usbfs);
    int release();

    int setStatusCallback(JNIEnv *env, jobject status_callback_obj);
    int setButtonCallback(JNIEnv *env, jobject button_callback_obj);

    char *getSupportedSize();
    int setPreviewSize(int width, int height, int min_fps, int max_fps, int mode, float bandwidth = DEFAULT_BANDWIDTH);
    int setRecordSize(int width, int height, int profile, int min_fps, int max_fps, int mode, float bandwidth = DEFAULT_BANDWIDTH);
    int setRecordSize(int width, int height, int profile, int usage, int min_fps, int max_fps, int mode, float bandwidth = DEFAULT_BANDWIDTH);
    int commitRecordSize(int width, int height, int profile, int usage, int min_fps, int max_fps, int mode, float bandwidth = DEFAULT_BANDWIDTH);
    int setPreviewDisplay(ANativeWindow *preview_window);
    int setFrameCallback(JNIEnv *env, jobject frame_callback_obj, int pixel_format);
    int startPreview();
    int stopPreview();
    int startRecord();
    int stopRecord();
    int setCaptureDisplay(ANativeWindow *capture_window);

    int getCtrlSupports(uint64_t *supports);
    int getProcSupports(uint64_t *supports);
    int getEncodeSupports(uint64_t *supports);
    int getEncodeRunningSupports(uint64_t *runningSupports);

    int updateScanningModeLimit(int &min, int &max, int &def);
    int setScanningMode(int mode);
    int getScanningMode();

    int updateExposureModeLimit(int &min, int &max, int &def);
    int setExposureMode(int mode);
    int getExposureMode();

    int updateExposurePriorityLimit(int &min, int &max, int &def);
    int setExposurePriority(int priority);
    int getExposurePriority();

    int updateExposureLimit(int &min, int &max, int &def);
    int setExposure(int ae_abs);
    int getExposure();

    int updateExposureRelLimit(int &min, int &max, int &def);
    int setExposureRel(int ae_rel);
    int getExposureRel();

    int updateAutoFocusLimit(int &min, int &max, int &def);
    int setAutoFocus(bool autoFocus);
    bool getAutoFocus();

    int updateFocusLimit(int &min, int &max, int &def);
    int setFocus(int focus);
    int getFocus();

    int updateFocusRelLimit(int &min, int &max, int &def);
    int setFocusRel(int focus);
    int getFocusRel();

    int updateIrisLimit(int &min, int &max, int &def);
    int setIris(int iris);
    int getIris();

    int updateIrisRelLimit(int &min, int &max, int &def);
    int setIrisRel(int iris);
    int getIrisRel();

    int updatePanLimit(int &min, int &max, int &def);
    int setPan(int pan);
    int getPan();

    int updateTiltLimit(int &min, int &max, int &def);
    int setTilt(int tilt);
    int getTilt();

    int updateRollLimit(int &min, int &max, int &def);
    int setRoll(int roll);
    int getRoll();

    int updatePanRelLimit(int &min, int &max, int &def);
    int setPanRel(int pan_rel);
    int getPanRel();

    int updateTiltRelLimit(int &min, int &max, int &def);
    int setTiltRel(int tilt_rel);
    int getTiltRel();

    int updateRollRelLimit(int &min, int &max, int &def);
    int setRollRel(int roll_rel);
    int getRollRel();

    int updatePrivacyLimit(int &min, int &max, int &def);
    int setPrivacy(int privacy);
    int getPrivacy();

    int updateAutoWhiteBlanceLimit(int &min, int &max, int &def);
    int setAutoWhiteBlance(bool autoWhiteBlance);
    bool getAutoWhiteBlance();

    int updateAutoWhiteBlanceCompoLimit(int &min, int &max, int &def);
    int setAutoWhiteBlanceCompo(bool autoWhiteBlanceCompo);
    bool getAutoWhiteBlanceCompo();

    int updateWhiteBlanceLimit(int &min, int &max, int &def);
    int setWhiteBlance(int temp);
    int getWhiteBlance();

    int updateWhiteBlanceCompoLimit(int &min, int &max, int &def);
    int setWhiteBlanceCompo(int white_blance_compo);
    int getWhiteBlanceCompo();

    int updateBacklightCompLimit(int &min, int &max, int &def);
    int setBacklightComp(int backlight);
    int getBacklightComp();

    int updateBrightnessLimit(int &min, int &max, int &def);
    int setBrightness(int brightness);
    int getBrightness();

    int updateContrastLimit(int &min, int &max, int &def);
    int setContrast(uint16_t contrast);
    int getContrast();

    int updateAutoContrastLimit(int &min, int &max, int &def);
    int setAutoContrast(bool autoFocus);
    bool getAutoContrast();

    int updateSharpnessLimit(int &min, int &max, int &def);
    int setSharpness(int sharpness);
    int getSharpness();

    int updateGainLimit(int &min, int &max, int &def);
    int setGain(int gain);
    int getGain();

    int updateGammaLimit(int &min, int &max, int &def);
    int setGamma(int gamma);
    int getGamma();

    int updateSaturationLimit(int &min, int &max, int &def);
    int setSaturation(int saturation);
    int getSaturation();

    int updateHueLimit(int &min, int &max, int &def);
    int setHue(int hue);
    int getHue();

    int updateAutoHueLimit(int &min, int &max, int &def);
    int setAutoHue(bool autoFocus);
    bool getAutoHue();

    int updatePowerlineFrequencyLimit(int &min, int &max, int &def);
    int setPowerlineFrequency(int frequency);
    int getPowerlineFrequency();

    int updateZoomLimit(int &min, int &max, int &def);
    int setZoom(int zoom);
    int getZoom();

    int updateZoomRelLimit(int &min, int &max, int &def);
    int setZoomRel(int zoom);
    int getZoomRel();

    int updateDigitalMultiplierLimit(int &min, int &max, int &def);
    int setDigitalMultiplier(int multiplier);
    int getDigitalMultiplier();

    int updateDigitalMultiplierLimitLimit(int &min, int &max, int &def);
    int setDigitalMultiplierLimit(int multiplier_limit);
    int getDigitalMultiplierLimit();

    int updateAnalogVideoStandardLimit(int &min, int &max, int &def);
    int setAnalogVideoStandard(int standard);
    int getAnalogVideoStandard();

    int updateAnalogVideoLockStateLimit(int &min, int &max, int &def);
    int setAnalogVideoLockState(int status);
    int getAnalogVideoLockState();

    int updateAverageBitrateLimit(int &min, int &max, int &def);
    int setAverageBitrate(int bitrate);
    int getAverageBitrate();

    int updateSyncRefFrameLimit(int &min, int &max, int &def);
    int setSyncRefFrame(int value);
    int getSyncRefFrame();

    int updateCPBSizeLimit(int &min, int &max, int &def);
    int setCPBSize(int value);
    int getCPBSize();

    int setSelectLayer(int value);
    int getSelectLayer();
};

#endif /* UVCCAMERA_H_ */
