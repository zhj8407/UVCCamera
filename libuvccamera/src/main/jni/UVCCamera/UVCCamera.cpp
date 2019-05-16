/*
 * UVCCamera
 * library and sample to access to UVC web camera on non-rooted Android device
 *
 * Copyright (c) 2014-2017 saki t_saki@serenegiant.com
 *
 * File name: UVCCamera.cpp
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

#define LOG_TAG "UVCCamera"
#if 1 // デバッグ情報を出さない時1
#ifndef LOG_NDEBUG
#define LOG_NDEBUG // LOGV/LOGD/MARKを出力しない時
#endif
#undef USE_LOGALL // 指定したLOGxだけを出力
#else
#define USE_LOGALL
#undef LOG_NDEBUG
#undef NDEBUG
#define GET_RAW_DESCRIPTOR
#endif

//**********************************************************************
//
//**********************************************************************
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <linux/time.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <assert.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <string.h>
#include <fcntl.h>
#include <dirent.h>

#include <memory>
#include <iostream>
#include <string>
#include <cstdio>

#include "UVCCamera.h"
#include "Parameters.h"
#include "libuvc_internal.h"

#include "v4l2_controls.h"

#define LOCAL_DEBUG 0

#define LINUX_V4L2_CLASS_SYSFS  "/sys/class/video4linux"

template<typename ... Args>
std::string stringFormat(const std::string &format,
                         Args ... args)
{
    size_t size = snprintf(nullptr, 0, format.c_str(), args ...) + 1;
    std::unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args ...);
    return std::string(buf.get(), buf.get() + size - 1);
}

template<typename ... Args>
int readFromFile(const std::string &filename,
                 const std::string &format,
                 Args ... args)
{
    int fd = 0;
    char buf[PATH_MAX];
    int len;

    fd = ::open(filename.c_str(), O_RDONLY);

    if (fd < 0)
    {
        LOGE("Can not open the file: %s, error: %s(%d)\n",
             filename.c_str(), strerror(errno), errno);
        return -1;
    }

    len = ::read(fd, buf, sizeof(buf));

    if (len <= 0)
    {
        LOGE("Can not read data from file: %s, error: %s(%d)\n",
             filename.c_str(), strerror(errno), errno);
        close(fd);
        return -1;
    }

    len = ::sscanf(buf, format.c_str(), args ...);

    if (len <= 0)
    {
        LOGE("Can not parse the value from %s\n", filename.c_str());
        close(fd);
        return -1;
    }

    ::close(fd);
    return len;
}

//**********************************************************************
//
//**********************************************************************
/**
 * コンストラクタ
 */
UVCCamera::UVCCamera()
    :  mStatusCallback(NULL),
       mButtonCallback(NULL),
       mPreview(NULL)
{

    ENTER();

    memset(mV4l2Devices, 0x00, UVC_MAX_DEVICES_NUM * sizeof(v4l2_dev_t *));

    EXIT();
}

UVCCamera::~UVCCamera()
{
    ENTER();
    release();
    EXIT();
}

void UVCCamera::getAvailableV4l2Devices(int vendorId,
                                        int productId,
                                        const std::string &serial,
                                        int busNum)
{
    struct dirent *de;
    DIR *dir;

    char linkname[PATH_MAX];
    int ret;

    dir = ::opendir(LINUX_V4L2_CLASS_SYSFS);

    if (dir == NULL)
    {
        return;
    }

    ::rewinddir(dir);

    while ((de = ::readdir(dir)) != NULL)
    {
        if (strcmp(de->d_name, ".") == 0 ||
                strcmp(de->d_name, "..") == 0)
        {
            continue;
        }

        /* We only care about the videoX device. */
        if (strncmp(de->d_name, "video", 5) != 0)
        {
            continue;
        }

        std::string v4l2SysFsName =
            stringFormat("%s/%s", LINUX_V4L2_CLASS_SYSFS, de->d_name);

        ret = ::readlink(v4l2SysFsName.c_str(),
                         linkname,
                         PATH_MAX);

        if (ret < 0 || ret >= PATH_MAX)
        {
            /* Can not get the link. */
            continue;
        }

        std::string linkNameStr(linkname);

        /* Confirm the usb[busnum] is in the path */
        std::string usbBus = stringFormat("usb%d", busNum);

        size_t keywordIndex = linkNameStr.find(usbBus);

        if (keywordIndex == std::string::npos)
        {
            /* No usb bus found. */
            continue;
        }

        keywordIndex = linkNameStr.find("/video4linux");

        if (keywordIndex == std::string::npos)
        {
            continue;
        }

        linkNameStr = linkNameStr.substr(0, keywordIndex);

        keywordIndex = linkNameStr.find_last_of("/");

        if (keywordIndex == std::string::npos)
        {
            continue;
        }

        std::string usbBusPath = linkNameStr.substr(0, keywordIndex);

        int vid, pid;
        char serialBuf[PATH_MAX];

        std::string usbVidPath =
            stringFormat("%s/%s/idVendor", LINUX_V4L2_CLASS_SYSFS, usbBusPath.c_str());
        ret = readFromFile(usbVidPath, "%x\n", &vid);

        if (ret <= 0 || vid != vendorId)
        {
            continue;
        }

        std::string usbPidPath =
            stringFormat("%s/%s/idProduct", LINUX_V4L2_CLASS_SYSFS, usbBusPath.c_str());
        ret = readFromFile(usbPidPath, "%x\n", &pid);

        if (ret <= 0 || pid != productId)
        {
            continue;
        }

        std::string usbSerialPath =
            stringFormat("%s/%s/serial", LINUX_V4L2_CLASS_SYSFS, usbBusPath.c_str());
        ret = readFromFile(usbSerialPath, "%s\n", serialBuf);

        if (ret <= 0 || std::string(serialBuf) != serial)
        {
            continue;
        }

        mCameraIds.push_back(stringFormat("/dev/%s", de->d_name));
    }

    ::closedir(dir);
}

//======================================================================

int UVCCamera::connect(int vid, int pid, int busnum, const char *serialNum)
{
    ENTER();
    uvc_error_t result = UVC_ERROR_BUSY;
    int tries = 5;

    do
    {
        /* Reset the cameras. */
        mCameraIds.clear();

        getAvailableV4l2Devices(vid, pid, std::string(serialNum), busnum);

        if (mCameraIds.size() >= 2)
        {
            if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] == nullptr)
            {
                mV4l2Devices[UVC_PREVIEW_DEVICE_ID] = v4l2core_init_dev(mCameraIds[UVC_PREVIEW_DEVICE_ID].c_str());

                if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] == nullptr)
                {
                    LOGE("%s: v4l2core_init_dev (%s) failed\n", __FUNCTION__, mCameraIds[UVC_PREVIEW_DEVICE_ID].c_str());
                    return UVC_ERROR_NO_DEVICE;
                }

                mPreview = new UVCPreview(mV4l2Devices[UVC_PREVIEW_DEVICE_ID]);
            }
            else
            {
                LOGW("%s: v4l2core_init_dev (%s) camera is already opened. you should release firstly.\n",
                     __FUNCTION__, mCameraIds[UVC_PREVIEW_DEVICE_ID].c_str());
            }

            if (mV4l2Devices[UVC_RECORD_DEVICE_ID] == nullptr)
            {
                mV4l2Devices[UVC_RECORD_DEVICE_ID] = v4l2core_init_dev(mCameraIds[UVC_RECORD_DEVICE_ID].c_str());

                if (mV4l2Devices[UVC_RECORD_DEVICE_ID] == nullptr)
                {
                    LOGE("%s: v4l2core_init_dev (%s) failed\n", __FUNCTION__, mCameraIds[UVC_RECORD_DEVICE_ID].c_str());
                    return UVC_ERROR_NO_DEVICE;
                }

                mRecord = new UVCRecord(mV4l2Devices[UVC_RECORD_DEVICE_ID]);
            }
            else
            {
                LOGW("%s: v4l2core_init_dev (%s) camera is already opened. you should release firstly.\n",
                     __FUNCTION__, mCameraIds[UVC_RECORD_DEVICE_ID].c_str());
            }

            result = UVC_SUCCESS;
        }
        else
        {
            result = UVC_ERROR_NO_DEVICE;
            usleep(100 * 1000); /* Sleep for 100ms. */
        }
    }
    while (result != UVC_SUCCESS && tries--);

    if (result != UVC_SUCCESS && (tries <= 0))
    {
        LOGE("%s: Can not find the UVC1.5 Camera! vid: 0x%04x, pid: 0x%04x, serial: %s\n",
             __FUNCTION__, vid, pid, serialNum);
    }

    RETURN(result, int);
}

int UVCCamera::release()
{
    ENTER();
    stopPreview();

    SAFE_DELETE(mStatusCallback);
    SAFE_DELETE(mButtonCallback);
    SAFE_DELETE(mPreview);
    SAFE_DELETE(mRecord);

    for (int i = 0; i < UVC_MAX_DEVICES_NUM; i++)
    {
        if (mV4l2Devices[i] != nullptr)
        {
            v4l2core_close_dev(mV4l2Devices[i]);
            mV4l2Devices[i] = nullptr;
        }
    }

    RETURN(0, int);
}

int UVCCamera::setStatusCallback(JNIEnv *env, jobject status_callback_obj)
{
    ENTER();
    int result = EXIT_FAILURE;

    if (mStatusCallback)
    {
        result = mStatusCallback->setCallback(env, status_callback_obj);
    }

    RETURN(result, int);
}

int UVCCamera::setButtonCallback(JNIEnv *env, jobject button_callback_obj)
{
    ENTER();
    int result = EXIT_FAILURE;

    if (mButtonCallback)
    {
        result = mButtonCallback->setCallback(env, button_callback_obj);
    }

    RETURN(result, int);
}

char *UVCCamera::getSupportedSize()
{
    ENTER();

#if 0

    if (mDeviceHandle)
    {
        UVCDiags params;
        RETURN(params.getSupportedSize(mDeviceHandle), char *)
    }

#endif

    RETURN(NULL, char *);
}

int UVCCamera::setPreviewSize(int width, int height, int min_fps, int max_fps, int mode, float bandwidth)
{
    ENTER();
    int result = EXIT_FAILURE;

    if (mPreview)
    {
        result = mPreview->setPreviewSize(width, height, min_fps, max_fps, mode, bandwidth);
    }

    RETURN(result, int);
}

int UVCCamera::setRecordSize(int width, int height, int profile, int min_fps, int max_fps, int mode, float bandwidth)
{
    ENTER();
    int result = EXIT_FAILURE;

    if (mRecord)
    {
        result = mRecord->setRecordSize(width, height, profile, min_fps, max_fps, mode, bandwidth);
    }

    RETURN(result, int);
}

int UVCCamera::setRecordSize(int width, int height, int profile, int usage, int min_fps, int max_fps, int mode, float bandwidth)
{
    ENTER();
    int result = EXIT_FAILURE;

    if (mRecord)
    {
        result = mRecord->setRecordSize(width, height, profile, usage, min_fps, max_fps, mode, bandwidth);
    }

    RETURN(result, int);
}

int UVCCamera::setPreviewDisplay(ANativeWindow *preview_window)
{
    ENTER();
    int result = EXIT_FAILURE;

    if (mPreview)
    {
        result = mPreview->setPreviewDisplay(preview_window);
    }

    RETURN(result, int);
}

int UVCCamera::setFrameCallback(JNIEnv *env, jobject frame_callback_obj, int pixel_format)
{
    ENTER();
    int result = EXIT_FAILURE;

    if (mPreview && pixel_format != 6)
    {
        result = mPreview->setFrameCallback(env, frame_callback_obj, pixel_format);
    }

    if (mRecord && pixel_format == 6)
    {
        result = mRecord->setFrameCallback(env, frame_callback_obj, pixel_format);
    }

    RETURN(result, int);
}

int UVCCamera::startPreview()
{
    ENTER();

    int result = EXIT_FAILURE;

    if (LIKELY(mV4l2Devices[UVC_PREVIEW_DEVICE_ID] && mPreview))
    {
        return mPreview->startPreview();
    }

    RETURN(result, int);
}

int UVCCamera::stopPreview()
{
    ENTER();

    if (LIKELY(mPreview))
    {
        mPreview->stopPreview();
    }

    RETURN(0, int);
}

int UVCCamera::startRecord()
{
    ENTER();

    int result = EXIT_FAILURE;

    if (LIKELY(mV4l2Devices[UVC_RECORD_DEVICE_ID] && mRecord))
    {
        return mRecord->startRecord();
    }

    RETURN(result, int);
}

int UVCCamera::stopRecord()
{
    ENTER();

    if (LIKELY(mRecord))
    {
        mRecord->stopRecord();
    }

    RETURN(0, int);
}

int UVCCamera::setCaptureDisplay(ANativeWindow *capture_window)
{
    ENTER();
    int result = EXIT_FAILURE;

    if (mPreview)
    {
        result = mPreview->setCaptureDisplay(capture_window);
    }

    RETURN(result, int);
}

//======================================================================
bool UVCCamera::getVideoControlSupported(const char *ctrlName, int deviceID)
{
    ENTER();

    int ret = false;

    if (deviceID < 0 || deviceID >= UVC_MAX_DEVICES_NUM)
    {
        return ret;
    }

    if (mV4l2Devices[deviceID] != NULL)
    {
        v4l2_dev_t *vd = mV4l2Devices[deviceID];

        ret = (get_control_by_name(vd, std::string(ctrlName)) != NULL);
    }

    RETURN(ret, bool);
}

//======================================================================
int UVCCamera::setVideoControlSetList(const char *ctrlSets, int deviceID)
{
    ENTER();

    int ret = -1;

    if (deviceID < 0 || deviceID >= UVC_MAX_DEVICES_NUM)
    {
        return ret;
    }

    if (mV4l2Devices[deviceID] != NULL)
    {
        v4l2_dev_t *vd = mV4l2Devices[deviceID];

        v4l2core_gen_ctrl_list(vd, std::string(ctrlSets));

        ret = 0;
    }

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateScanningModeLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

int UVCCamera::setScanningMode(int mode)
{
    ENTER();

    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

int UVCCamera::getScanningMode()
{

    ENTER();

    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

//======================================================================
int UVCCamera::updateExposureModeLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_EXPOSURE_AUTO, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setExposureMode(int mode)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_EXPOSURE_AUTO, mode);

    RETURN(ret, int);
}

int UVCCamera::getExposureMode()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_EXPOSURE_AUTO);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateExposurePriorityLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_EXPOSURE_AUTO_PRIORITY, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setExposurePriority(int priority)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_EXPOSURE_AUTO_PRIORITY, priority);

    RETURN(ret, int);
}

int UVCCamera::getExposurePriority()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_EXPOSURE_AUTO_PRIORITY);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateExposureLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_EXPOSURE_ABSOLUTE, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setExposure(int ae_abs)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_EXPOSURE_ABSOLUTE, ae_abs);

    RETURN(ret, int);
}

int UVCCamera::getExposure()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_EXPOSURE_ABSOLUTE);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateExposureRelLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::setExposureRel(int ae_rel)
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

int UVCCamera::getExposureRel()
{

    ENTER();
    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

//======================================================================
int UVCCamera::updateAutoFocusLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_FOCUS_AUTO, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setAutoFocus(bool autoFocus)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_FOCUS_AUTO, autoFocus ? 1 : 0);

    RETURN(ret, int);
}

bool UVCCamera::getAutoFocus()
{
    ENTER();

    bool ret = (getUVCControlValue(V4L2_CID_FOCUS_AUTO) == 1);

    RETURN(ret, bool);
}

//======================================================================
int UVCCamera::updateFocusLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_FOCUS_ABSOLUTE, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setFocus(int focus)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_FOCUS_ABSOLUTE, focus);

    RETURN(ret, int);
}

int UVCCamera::getFocus()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_FOCUS_ABSOLUTE);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateFocusRelLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

int UVCCamera::setFocusRel(int focus_rel)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

int UVCCamera::getFocusRel()
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(0, int);
}

//======================================================================
int UVCCamera::updateIrisLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_IRIS_ABSOLUTE, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setIris(int iris)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_IRIS_ABSOLUTE, iris);

    RETURN(ret, int);
}

int UVCCamera::getIris()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_IRIS_ABSOLUTE);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateIrisRelLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_IRIS_RELATIVE, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setIrisRel(int iris_rel)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_IRIS_RELATIVE, iris_rel);

    RETURN(ret, int);
}

int UVCCamera::getIrisRel()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_IRIS_RELATIVE);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updatePanLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_PAN_ABSOLUTE, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setPan(int pan)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_PAN_ABSOLUTE, pan);

    RETURN(ret, int);
}

int UVCCamera::getPan()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_PAN_ABSOLUTE);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateTiltLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_TILT_ABSOLUTE, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setTilt(int tilt)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_TILT_ABSOLUTE, tilt);

    RETURN(ret, int);
}

int UVCCamera::getTilt()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_TILT_ABSOLUTE);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateRollLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

int UVCCamera::setRoll(int roll)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

int UVCCamera::getRoll()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
int UVCCamera::updatePanRelLimit(int &min, int &max, int &def)
{
    ENTER();
    // FIXME not implemented yet
    RETURN(UVC_ERROR_ACCESS, int);
}

int UVCCamera::setPanRel(int pan_rel)
{
    ENTER();
    // FIXME not implemented yet
    RETURN(UVC_ERROR_ACCESS, int);
}

int UVCCamera::getPanRel()
{
    ENTER();
    // FIXME not implemented yet
    RETURN(UVC_ERROR_ACCESS, int);
}

//======================================================================
int UVCCamera::updateTiltRelLimit(int &min, int &max, int &def)
{
    ENTER();
    // FIXME not implemented yet
    RETURN(UVC_ERROR_ACCESS, int);
}

int UVCCamera::setTiltRel(int tilt_rel)
{
    ENTER();
    // FIXME not implemented yet
    RETURN(UVC_ERROR_ACCESS, int);
}

int UVCCamera::getTiltRel()
{
    ENTER();
    // FIXME not implemented yet
    RETURN(UVC_ERROR_ACCESS, int);
}

//======================================================================
int UVCCamera::updateRollRelLimit(int &min, int &max, int &def)
{
    ENTER();
    // FIXME not implemented yet
    RETURN(UVC_ERROR_ACCESS, int);
}

int UVCCamera::setRollRel(int roll_rel)
{
    ENTER();
    // FIXME not implemented yet
    RETURN(UVC_ERROR_ACCESS, int);
}

int UVCCamera::getRollRel()
{
    ENTER();
    // FIXME not implemented yet
    RETURN(UVC_ERROR_ACCESS, int);
}

//======================================================================
int UVCCamera::updatePrivacyLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_PRIVACY, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setPrivacy(int privacy)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_PRIVACY, privacy);

    RETURN(ret, int);
}

int UVCCamera::getPrivacy()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_PRIVACY);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateBacklightCompLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::setBacklightComp(int backlight)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::getBacklightComp()
{
    ENTER();

    RETURN(0, int);
}

int UVCCamera::updateBrightnessLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_BRIGHTNESS, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setBrightness(int brightness)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_BRIGHTNESS, brightness);

    RETURN(ret, int);
}

int UVCCamera::getBrightness()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_BRIGHTNESS);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateContrastLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_CONTRAST, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setContrast(uint16_t contrast)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_CONTRAST, contrast);

    RETURN(ret, int);
}

int UVCCamera::getContrast()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_CONTRAST);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateAutoContrastLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::setAutoContrast(bool autoContrast)
{
    ENTER();

    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

bool UVCCamera::getAutoContrast()
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

//======================================================================
int UVCCamera::updateSharpnessLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_SHARPNESS, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setSharpness(int sharpness)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_SHARPNESS, sharpness);

    RETURN(ret, int);
}

int UVCCamera::getSharpness()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_SHARPNESS);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateGainLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_GAIN, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setGain(int gain)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_GAIN, gain);

    RETURN(ret, int);
}

int UVCCamera::getGain()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_GAIN);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateAutoWhiteBlanceLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_AUTO_WHITE_BALANCE, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setAutoWhiteBlance(bool autoWhiteBlance)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_AUTO_WHITE_BALANCE, autoWhiteBlance ? 1 : 0);

    RETURN(ret, int);
}

bool UVCCamera::getAutoWhiteBlance()
{
    ENTER();

    bool ret = (getUVCControlValue(V4L2_CID_AUTO_WHITE_BALANCE) == 1);

    RETURN(ret, bool);
}

//======================================================================

int UVCCamera::updateAutoWhiteBlanceCompoLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_AUTO_WHITE_BALANCE, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setAutoWhiteBlanceCompo(bool autoWhiteBlanceCompo)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_AUTO_WHITE_BALANCE, autoWhiteBlanceCompo ? 1 : 0);

    RETURN(ret, int);
}

bool UVCCamera::getAutoWhiteBlanceCompo()
{
    ENTER();

    bool ret = (getUVCControlValue(V4L2_CID_AUTO_WHITE_BALANCE) == 1);

    RETURN(ret, bool);
}

//======================================================================
int UVCCamera::updateWhiteBlanceLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_WHITE_BALANCE_TEMPERATURE, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setWhiteBlance(int white_blance)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_WHITE_BALANCE_TEMPERATURE, white_blance);

    RETURN(ret, int);
}

int UVCCamera::getWhiteBlance()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_WHITE_BALANCE_TEMPERATURE);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateWhiteBlanceCompoLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;


    RETURN(ret, int);
}

int UVCCamera::setWhiteBlanceCompo(int white_blance_compo)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::getWhiteBlanceCompo()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
int UVCCamera::updateGammaLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_GAMMA, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setGamma(int gamma)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_GAMMA, gamma);

    RETURN(ret, int);
}

int UVCCamera::getGamma()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_GAMMA);

    RETURN(ret, int);
}

//======================================================================

int UVCCamera::updateSaturationLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_SATURATION, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setSaturation(int saturation)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_SATURATION, saturation);

    RETURN(ret, int);
}

int UVCCamera::getSaturation()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_SATURATION);

    RETURN(ret, int);
}

//======================================================================
// 色相調整
int UVCCamera::updateHueLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_HUE, min, max, def);

    RETURN(ret, int);
}

// 色相を設定
int UVCCamera::setHue(int hue)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_HUE, hue);

    RETURN(ret, int);
}

// 色相の現在値を取得
int UVCCamera::getHue()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_HUE);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateAutoHueLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_HUE_AUTO, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setAutoHue(bool autoHue)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_HUE_AUTO, autoHue ? 1 : 0);

    RETURN(ret, int);
}

bool UVCCamera::getAutoHue()
{
    ENTER();

    bool ret = (getUVCControlValue(V4L2_CID_HUE_AUTO) == 1);

    RETURN(ret, bool);
}

//======================================================================
int UVCCamera::updatePowerlineFrequencyLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_POWER_LINE_FREQUENCY, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setPowerlineFrequency(int frequency)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_POWER_LINE_FREQUENCY, frequency);

    RETURN(ret, int);
}

int UVCCamera::getPowerlineFrequency()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_POWER_LINE_FREQUENCY);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateZoomLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_ZOOM_ABSOLUTE, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setZoom(int zoom)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_ZOOM_ABSOLUTE, zoom);

    RETURN(ret, int);
}

int UVCCamera::getZoom()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_ZOOM_ABSOLUTE);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateZoomRelLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_ZOOM_RELATIVE, min, max, def);

    RETURN(ret, int);
}

int UVCCamera::setZoomRel(int zoom)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_ZOOM_RELATIVE, zoom);

    RETURN(ret, int);
}

int UVCCamera::getZoomRel()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_ZOOM_RELATIVE);

    RETURN(ret, int);
}

//======================================================================
int UVCCamera::updateDigitalMultiplierLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::setDigitalMultiplier(int multiplier)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::getDigitalMultiplier()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
int UVCCamera::updateDigitalMultiplierLimitLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::setDigitalMultiplierLimit(int multiplier_limit)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::getDigitalMultiplierLimit()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
int UVCCamera::updateAnalogVideoStandardLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::setAnalogVideoStandard(int standard)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::getAnalogVideoStandard()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
int UVCCamera::updateAnalogVideoLockStateLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::setAnalogVideoLockState(int state)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::getAnalogVideoLockState()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// AverateBitrateStatus
int UVCCamera::updateAverageBitrateLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_ENCODER_AVERAGE_BITRATE,
                                    min, max, def, UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

int UVCCamera::setAverageBitrate(int bitrate)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_ENCODER_AVERAGE_BITRATE,
                                 bitrate, UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

int UVCCamera::getAverageBitrate()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_ENCODER_AVERAGE_BITRATE,
                                 UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

//======================================================================
// SyncRefFrameStatus
int UVCCamera::updateSyncRefFrameLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_ENCODER_VP8_SYNC_FRAME_TYPE,
                                    min, max, def, UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

int UVCCamera::setSyncRefFrame(int value)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_ENCODER_VP8_SYNC_FRAME_TYPE,
                                 value, UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

int UVCCamera::getSyncRefFrame()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_ENCODER_VP8_SYNC_FRAME_TYPE,
                                 UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

//======================================================================
// SyncRefFrameIntervalStatus
int UVCCamera::updateSyncRefFrameIntervalLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_ENCODER_SYNC_FRAME_INTERVAL,
                                    min, max, def, UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

int UVCCamera::setSyncRefFrameInterval(int value)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_ENCODER_SYNC_FRAME_INTERVAL,
                                 value, UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

int UVCCamera::getSyncRefFrameInterval()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_ENCODER_SYNC_FRAME_INTERVAL,
                                 UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

//======================================================================
// CPBSizeStatus
int UVCCamera::updateCPBSizeLimit(int &min, int &max, int &def)
{
    ENTER();

    int ret = updateUVCControlLimit(V4L2_CID_ENCODER_CPB_SIZE,
                                    min, max, def, UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

int UVCCamera::setCPBSize(int value)
{
    ENTER();

    int ret = setUVCControlValue(V4L2_CID_ENCODER_CPB_SIZE,
                                 value, UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

int UVCCamera::getCPBSize()
{
    ENTER();

    int ret = getUVCControlValue(V4L2_CID_ENCODER_CPB_SIZE,
                                 UVC_RECORD_DEVICE_ID);

    RETURN(ret, int);
}

int UVCCamera::setSelectLayer(int value)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::getSelectLayer()
{
    ENTER();

    RETURN(0, int);
}