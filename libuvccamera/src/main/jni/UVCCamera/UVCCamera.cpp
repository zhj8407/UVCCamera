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
#include <linux/time.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <assert.h>

#include <sys/ioctl.h>
#include <sys/types.h>

#include "UVCCamera.h"
#include "Parameters.h"
#include "libuvc_internal.h"

#include "v4l2_controls.h"

#define LOCAL_DEBUG 0

//**********************************************************************
//
//**********************************************************************
/**
 * コンストラクタ
 */
UVCCamera::UVCCamera()
    : mContext(NULL),
      mDevice(NULL),
      mDeviceHandle(NULL),
      mStatusCallback(NULL),
      mButtonCallback(NULL),
      mPreview(NULL),
      mCtrlSupports(0),
      mPUSupports(0),
      mEUSupports(0),
      mEURuntimeSupports(0)
{

    ENTER();
    mCameraIds[UVC_PREVIEW_DEVICE_ID] = "/dev/video0";
    mCameraIds[UVC_RECORD_DEVICE_ID] = "/dev/video1";

    memset(mV4l2Devices, 0x00, UVC_MAX_DEVICES_NUM * sizeof(v4l2_dev_t *));

    EXIT();
}

UVCCamera::~UVCCamera()
{
    ENTER();
    release();

    if (mContext)
    {
        uvc_exit(mContext);
        mContext = NULL;
    }

    EXIT();
}


//======================================================================

int UVCCamera::connect(int vid, int pid, int fd, int busnum, int devaddr, const char *usbfs)
{
    ENTER();
    uvc_error_t result = UVC_ERROR_BUSY;

#if 0

    if (!mDeviceHandle && fd)
    {
        if (mUsbFs)
        {
            free(mUsbFs);
        }

        mUsbFs = strdup(usbfs);

        if (UNLIKELY(!mContext))
        {
            result = uvc_init2(&mContext, NULL, mUsbFs);

            //libusb_set_debug(mContext->usb_ctx, LIBUSB_LOG_LEVEL_DEBUG);
            if (UNLIKELY(result < 0))
            {
                LOGD("failed to init libuvc");
                RETURN(result, int);
            }
        }

        clearCameraParams();
        fd = dup(fd);
        result = uvc_get_device_with_fd(mContext, &mDevice, vid, pid, NULL, fd, busnum, devaddr);

        if (LIKELY(!result))
        {

            result = uvc_open(mDevice, &mDeviceHandle);

            if (LIKELY(!result))
            {

#if LOCAL_DEBUG
                uvc_print_diag(mDeviceHandle, stderr);
#endif
                mFd = fd;
                mStatusCallback = new UVCStatusCallback(mDeviceHandle);
                mButtonCallback = new UVCButtonCallback(mDeviceHandle);
                mPreview = new UVCPreview(mDeviceHandle);
                mRecord = new UVCRecord(mDeviceHandle);
            }
            else
            {

                LOGE("could not open camera:err=%d", result);
                uvc_unref_device(mDevice);
                mDevice = NULL;
                mDeviceHandle = NULL;
                close(fd);
            }
        }
        else
        {
            LOGE("could not find camera:err=%d", result);
            close(fd);
        }
    }
    else
    {
        LOGW("camera is already opened. you should release first");
    }

#else

    if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] == nullptr)
    {
        mV4l2Devices[UVC_PREVIEW_DEVICE_ID] = v4l2core_init_dev(mCameraIds[UVC_PREVIEW_DEVICE_ID].c_str());

        if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] == nullptr)
        {
            LOGE("%s: v4l2core_init_dev (%s) failed\n", __FUNCTION__, mCameraIds[UVC_PREVIEW_DEVICE_ID].c_str());
            return UVC_ERROR_NO_DEVICE;
        }

        mPreview = new UVCPreview(mDeviceHandle, mV4l2Devices[UVC_PREVIEW_DEVICE_ID]);
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

        mRecord = new UVCRecord(mDeviceHandle, mV4l2Devices[UVC_RECORD_DEVICE_ID]);
    }
    else
    {
        LOGW("%s: v4l2core_init_dev (%s) camera is already opened. you should release firstly.\n",
             __FUNCTION__, mCameraIds[UVC_RECORD_DEVICE_ID].c_str());
    }

    result = UVC_SUCCESS;
#endif

    RETURN(result, int);
}

// カメラを開放する
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

    if (mDeviceHandle)
    {
        UVCDiags params;
        RETURN(params.getSupportedSize(mDeviceHandle), char *)
    }

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

int UVCCamera::commitRecordSize(int width, int height, int profile, int usage, int min_fps, int max_fps, int mode, float bandwidth)
{
    ENTER();
    int result = EXIT_FAILURE;

    if (mRecord)
    {
        result = mRecord->commitRecordSize(width, height, profile, usage, min_fps, max_fps, mode, bandwidth);
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
// カメラのサポートしているコントロール機能を取得する
int UVCCamera::getCtrlSupports(uint64_t *supports)
{
    ENTER();

    uvc_error_t ret = UVC_SUCCESS;
    *supports = (uint64_t)1;

    RETURN(ret, int);
}

int UVCCamera::getProcSupports(uint64_t *supports)
{
    ENTER();

    uvc_error_t ret = UVC_SUCCESS;
    *supports = (uint64_t)0x03;

    RETURN(ret, int);
}

int UVCCamera::getEncodeSupports(uint64_t *supports)
{
    ENTER();

    uvc_error_t ret = UVC_SUCCESS;
    *supports = (uint64_t)1;

    RETURN(ret, int);
}

int UVCCamera::getEncodeRunningSupports(uint64_t *runningSupports)
{
    ENTER();

    uvc_error_t ret = UVC_SUCCESS;
    *runningSupports = (uint64_t)1;

    RETURN(ret, int);
}

//======================================================================
#define CTRL_BRIGHTNESS 0
#define CTRL_CONTRAST 1
#define CTRL_SHARPNESS 2
#define CTRL_GAIN 3
#define CTRL_WHITEBLANCE 4
#define CTRL_FOCUS 5

//======================================================================
// スキャニングモード
int UVCCamera::updateScanningModeLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// スキャニングモードをセット
int UVCCamera::setScanningMode(int mode)
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_SCANNING))
    {
        //      LOGI("ae:%d", mode);
        r = uvc_set_scanning_mode(mDeviceHandle, mode /* & 0xff*/);
    }

    RETURN(r, int);
}

// スキャニングモード設定を取得
int UVCCamera::getScanningMode()
{

    ENTER();
    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_SCANNING))
    {
        uint8_t mode;
        r = uvc_get_scanning_mode(mDeviceHandle, &mode, UVC_GET_CUR);

        //      LOGI("ae:%d", mode);
        if (LIKELY(!r))
        {
            r = mode;
        }
    }

    RETURN(r, int);
}

//======================================================================
// 露出モード
int UVCCamera::updateExposureModeLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// 露出をセット
int UVCCamera::setExposureMode(int mode)
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_AE))
    {
        //      LOGI("ae:%d", mode);
        r = uvc_set_ae_mode(mDeviceHandle, mode /* & 0xff*/);
    }

    RETURN(r, int);
}

// 露出設定を取得
int UVCCamera::getExposureMode()
{

    ENTER();
    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_AE))
    {
        uint8_t mode;
        r = uvc_get_ae_mode(mDeviceHandle, &mode, UVC_GET_CUR);

        //      LOGI("ae:%d", mode);
        if (LIKELY(!r))
        {
            r = mode;
        }
    }

    RETURN(r, int);
}

//======================================================================
// 露出優先設定
int UVCCamera::updateExposurePriorityLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// 露出優先設定をセット
int UVCCamera::setExposurePriority(int priority)
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_AE_PRIORITY))
    {
        //      LOGI("ae priority:%d", priority);
        r = uvc_set_ae_priority(mDeviceHandle, priority /* & 0xff*/);
    }

    RETURN(r, int);
}

// 露出優先設定を取得
int UVCCamera::getExposurePriority()
{

    ENTER();
    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_AE_PRIORITY))
    {
        uint8_t priority;
        r = uvc_get_ae_priority(mDeviceHandle, &priority, UVC_GET_CUR);

        //      LOGI("ae priority:%d", priority);
        if (LIKELY(!r))
        {
            r = priority;
        }
    }

    RETURN(r, int);
}

//======================================================================
// 露出(絶対値)設定
int UVCCamera::updateExposureLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// 露出(絶対値)設定をセット
int UVCCamera::setExposure(int ae_abs)
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_AE_ABS))
    {
        //      LOGI("ae_abs:%d", ae_abs);
        r = uvc_set_exposure_abs(mDeviceHandle, ae_abs /* & 0xff*/);
    }

    RETURN(r, int);
}

// 露出(絶対値)設定を取得
int UVCCamera::getExposure()
{

    ENTER();
    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_AE_ABS))
    {
        int ae_abs;
        r = uvc_get_exposure_abs(mDeviceHandle, &ae_abs, UVC_GET_CUR);

        //      LOGI("ae_abs:%d", ae_abs);
        if (LIKELY(!r))
        {
            r = ae_abs;
        }
    }

    RETURN(r, int);
}

//======================================================================
// 露出(相対値)設定
int UVCCamera::updateExposureRelLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// 露出(相対値)設定をセット
int UVCCamera::setExposureRel(int ae_rel)
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_AE_REL))
    {
        //      LOGI("ae_rel:%d", ae_rel);
        r = uvc_set_exposure_rel(mDeviceHandle, ae_rel /* & 0xff*/);
    }

    RETURN(r, int);
}

// 露出(相対値)設定を取得
int UVCCamera::getExposureRel()
{

    ENTER();
    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_AE_REL))
    {
        int ae_rel;
        r = uvc_get_exposure_rel(mDeviceHandle, &ae_rel, UVC_GET_CUR);

        //      LOGI("ae_rel:%d", ae_rel);
        if (LIKELY(!r))
        {
            r = ae_rel;
        }
    }

    RETURN(r, int);
}

//======================================================================
// オートフォーカス
int UVCCamera::updateAutoFocusLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// オートフォーカスをon/off
int UVCCamera::setAutoFocus(bool autoFocus)
{
    ENTER();

    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_FOCUS_AUTO))
    {
        r = uvc_set_focus_auto(mDeviceHandle, autoFocus);
    }

    RETURN(r, int);
}

// オートフォーカスのon/off状態を取得
bool UVCCamera::getAutoFocus()
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    if
    LIKELY((mDeviceHandle) && (mCtrlSupports & CTRL_FOCUS_AUTO))
    {
        uint8_t autoFocus;
        r = uvc_get_focus_auto(mDeviceHandle, &autoFocus, UVC_GET_CUR);

        if (LIKELY(!r))
        {
            r = autoFocus;
        }
    }

    RETURN(r, int);
}

//======================================================================
// フォーカス(絶対値)調整
int UVCCamera::updateFocusLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// フォーカス(絶対値)を設定
int UVCCamera::setFocus(int focus)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    if (mCtrlSupports & CTRL_FOCUS_ABS)
    {
    }

    RETURN(ret, int);
}

// フォーカス(絶対値)の現在値を取得
int UVCCamera::getFocus()
{
    ENTER();

    if (mCtrlSupports & CTRL_FOCUS_ABS)
    {
    }

    RETURN(0, int);
}

//======================================================================
// フォーカス(相対値)調整
int UVCCamera::updateFocusRelLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// フォーカス(相対値)を設定
int UVCCamera::setFocusRel(int focus_rel)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    if (mCtrlSupports & CTRL_FOCUS_REL)
    {
    }

    RETURN(ret, int);
}

// フォーカス(相対値)の現在値を取得
int UVCCamera::getFocusRel()
{
    ENTER();

    if (mCtrlSupports & CTRL_FOCUS_REL)
    {
    }

    RETURN(0, int);
}

//======================================================================
// 絞り(絶対値)調整
int UVCCamera::updateIrisLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// 絞り(絶対値)を設定
int UVCCamera::setIris(int iris)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    if (mCtrlSupports & CTRL_IRIS_ABS)
    {
    }

    RETURN(ret, int);
}

// 絞り(絶対値)の現在値を取得
int UVCCamera::getIris()
{
    ENTER();

    if (mCtrlSupports & CTRL_IRIS_ABS)
    {
    }

    RETURN(0, int);
}

//======================================================================
// 絞り(相対値)調整
int UVCCamera::updateIrisRelLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// 絞り(相対値)を設定
int UVCCamera::setIrisRel(int iris_rel)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    if (mCtrlSupports & CTRL_IRIS_REL)
    {
    }

    RETURN(ret, int);
}

// 絞り(相対値)の現在値を取得
int UVCCamera::getIrisRel()
{
    ENTER();

    if (mCtrlSupports & CTRL_IRIS_REL)
    {
    }

    RETURN(0, int);
}

//======================================================================
// Pan(絶対値)調整
int UVCCamera::updatePanLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// Pan(絶対値)を設定
int UVCCamera::setPan(int pan)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// Pan(絶対値)の現在値を取得
int UVCCamera::getPan()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// Tilt(絶対値)調整
int UVCCamera::updateTiltLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// Tilt(絶対値)を設定
int UVCCamera::setTilt(int tilt)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// Tilt(絶対値)の現在値を取得
int UVCCamera::getTilt()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// Roll(絶対値)調整
int UVCCamera::updateRollLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// Roll(絶対値)を設定
int UVCCamera::setRoll(int roll)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// Roll(絶対値)の現在値を取得
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
// プライバシーモード
int UVCCamera::updatePrivacyLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// プライバシーモードを設定
int UVCCamera::setPrivacy(int privacy)
{
    ENTER();
    int ret = UVC_ERROR_ACCESS;

    RETURN(ret, int);
}

// プライバシーモードの現在値を取得
int UVCCamera::getPrivacy()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
/*
    // DigitalWindow
    int UVCCamera::updateDigitalWindowLimit(...not defined...) {
        ENTER();
        // FIXME not implemented yet
        RETURN(UVC_ERROR_ACCESS, int);
    }

    // DigitalWindowを設定
    int UVCCamera::setDigitalWindow(int top, int reft, int bottom, int right) {
        ENTER();
        // FIXME not implemented yet
        RETURN(UVC_ERROR_ACCESS, int);
    }

    // DigitalWindowの現在値を取得
    int UVCCamera::getDigitalWindow(int &top, int &reft, int &bottom, int &right) {
        ENTER();
        // FIXME not implemented yet
        RETURN(UVC_ERROR_ACCESS, int);
    }
    */

//======================================================================
/*
    // DigitalRoi
    int UVCCamera::updateDigitalRoiLimit(...not defined...) {
        ENTER();
        // FIXME not implemented yet
        RETURN(UVC_ERROR_ACCESS, int);
    }

    // DigitalRoiを設定
    int UVCCamera::setDigitalRoi(int top, int reft, int bottom, int right) {
        ENTER();
        // FIXME not implemented yet
        RETURN(UVC_ERROR_ACCESS, int);
    }

    // DigitalRoiの現在値を取得
    int UVCCamera::getDigitalRoi(int &top, int &reft, int &bottom, int &right) {
        ENTER();
        // FIXME not implemented yet
        RETURN(UVC_ERROR_ACCESS, int);
    }
    */

//======================================================================
// backlight_compensation
int UVCCamera::updateBacklightCompLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// backlight_compensationを設定
int UVCCamera::setBacklightComp(int backlight)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// backlight_compensationの現在値を取得
int UVCCamera::getBacklightComp()
{
    ENTER();

    RETURN(0, int);
}

int UVCCamera::updateBrightnessLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] != NULL)
    {
        v4l2_dev_t *vd = mV4l2Devices[UVC_PREVIEW_DEVICE_ID];

        v4l2_ctrl_t *control = get_control_by_id(vd, V4L2_CID_BRIGHTNESS);

        if (control != NULL)
        {
            min = control->control.minimum;
            max = control->control.maximum;
            def = control->control.default_value;

            ret = UVC_SUCCESS;
        }
    }

    RETURN(ret, int);
}

int UVCCamera::setBrightness(int brightness)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] != NULL)
    {
        v4l2_dev_t *vd = mV4l2Devices[UVC_PREVIEW_DEVICE_ID];

        v4l2_ctrl_t *control = get_control_by_id(vd, V4L2_CID_BRIGHTNESS);

        if (control != NULL)
        {
            control->value = brightness;

            ret = set_control_value_by_id(vd, V4L2_CID_BRIGHTNESS);
        }
    }

    RETURN(ret, int);
}

// 明るさの現在値を取得
int UVCCamera::getBrightness()
{
    ENTER();
    int ret = UVC_ERROR_IO;

    if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] != NULL)
    {
        v4l2_dev_t *vd = mV4l2Devices[UVC_PREVIEW_DEVICE_ID];

        if (!get_control_value_by_id(vd, V4L2_CID_BRIGHTNESS))
        {
            v4l2_ctrl_t *control = get_control_by_id(vd, V4L2_CID_BRIGHTNESS);

            ret = control->value;
        }
    }

    RETURN(ret, int);
}

//======================================================================
// コントラスト調整
int UVCCamera::updateContrastLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] != NULL)
    {
        v4l2_dev_t *vd = mV4l2Devices[UVC_PREVIEW_DEVICE_ID];

        v4l2_ctrl_t *control = get_control_by_id(vd, V4L2_CID_CONTRAST);

        if (control != NULL)
        {
            min = control->control.minimum;
            max = control->control.maximum;
            def = control->control.default_value;

            ret = UVC_SUCCESS;
        }
    }

    RETURN(ret, int);
}

// コントラストを設定
int UVCCamera::setContrast(uint16_t contrast)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] != NULL)
    {
        v4l2_dev_t *vd = mV4l2Devices[UVC_PREVIEW_DEVICE_ID];

        v4l2_ctrl_t *control = get_control_by_id(vd, V4L2_CID_CONTRAST);

        if (control != NULL)
        {
            control->value = contrast;

            ret = set_control_value_by_id(vd, V4L2_CID_CONTRAST);
        }
    }

    RETURN(ret, int);
}

// コントラストの現在値を取得
int UVCCamera::getContrast()
{
    ENTER();
    int ret = UVC_ERROR_IO;

    if (mV4l2Devices[UVC_PREVIEW_DEVICE_ID] != NULL)
    {
        v4l2_dev_t *vd = mV4l2Devices[UVC_PREVIEW_DEVICE_ID];

        if (!get_control_value_by_id(vd, V4L2_CID_CONTRAST))
        {
            v4l2_ctrl_t *control = get_control_by_id(vd, V4L2_CID_CONTRAST);

            ret = control->value;
        }
    }

    RETURN(ret, int);
}

//======================================================================
// オートコントラスト
int UVCCamera::updateAutoContrastLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// オートコントラストをon/off
int UVCCamera::setAutoContrast(bool autoContrast)
{
    ENTER();

    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

// オートコントラストのon/off状態を取得
bool UVCCamera::getAutoContrast()
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

//======================================================================
// シャープネス調整
int UVCCamera::updateSharpnessLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// シャープネスを設定
int UVCCamera::setSharpness(int sharpness)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// シャープネスの現在値を取得
int UVCCamera::getSharpness()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// ゲイン調整
int UVCCamera::updateGainLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// ゲインを設定
int UVCCamera::setGain(int gain)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// ゲインの現在値を取得
int UVCCamera::getGain()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// オートホワイトバランス(temp)
int UVCCamera::updateAutoWhiteBlanceLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// オートホワイトバランス(temp)をon/off
int UVCCamera::setAutoWhiteBlance(bool autoWhiteBlance)
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

// オートホワイトバランス(temp)のon/off状態を取得
bool UVCCamera::getAutoWhiteBlance()
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

//======================================================================
// オートホワイトバランス(compo)
int UVCCamera::updateAutoWhiteBlanceCompoLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// オートホワイトバランス(compo)をon/off
int UVCCamera::setAutoWhiteBlanceCompo(bool autoWhiteBlanceCompo)
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

// オートホワイトバランス(compo)のon/off状態を取得
bool UVCCamera::getAutoWhiteBlanceCompo()
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

//======================================================================
// ホワイトバランス色温度調整
int UVCCamera::updateWhiteBlanceLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;


    RETURN(ret, int);
}

// ホワイトバランス色温度を設定
int UVCCamera::setWhiteBlance(int white_blance)
{
    ENTER();
    int ret = UVC_ERROR_IO;


    RETURN(ret, int);
}

// ホワイトバランス色温度の現在値を取得
int UVCCamera::getWhiteBlance()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// ホワイトバランスcompo調整
int UVCCamera::updateWhiteBlanceCompoLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;


    RETURN(ret, int);
}

// ホワイトバランスcompoを設定
int UVCCamera::setWhiteBlanceCompo(int white_blance_compo)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// ホワイトバランスcompoの現在値を取得
int UVCCamera::getWhiteBlanceCompo()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// ガンマ調整
int UVCCamera::updateGammaLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// ガンマを設定
int UVCCamera::setGamma(int gamma)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// ガンマの現在値を取得
int UVCCamera::getGamma()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// 彩度調整
int UVCCamera::updateSaturationLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// 彩度を設定
int UVCCamera::setSaturation(int saturation)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// 彩度の現在値を取得
int UVCCamera::getSaturation()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// 色相調整
int UVCCamera::updateHueLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// 色相を設定
int UVCCamera::setHue(int hue)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// 色相の現在値を取得
int UVCCamera::getHue()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// オート色相
int UVCCamera::updateAutoHueLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// オート色相をon/off
int UVCCamera::setAutoHue(bool autoHue)
{
    ENTER();

    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

// オート色相のon/off状態を取得
bool UVCCamera::getAutoHue()
{
    ENTER();
    int r = UVC_ERROR_ACCESS;

    RETURN(r, int);
}

//======================================================================
// 電源周波数によるチラつき補正
int UVCCamera::updatePowerlineFrequencyLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// 電源周波数によるチラつき補正を設定
int UVCCamera::setPowerlineFrequency(int frequency)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// 電源周波数によるチラつき補正値を取得
int UVCCamera::getPowerlineFrequency()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// ズーム(abs)調整
int UVCCamera::updateZoomLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// ズーム(abs)を設定
int UVCCamera::setZoom(int zoom)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// ズーム(abs)の現在値を取得
int UVCCamera::getZoom()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// ズーム(相対値)調整
int UVCCamera::updateZoomRelLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// ズーム(相対値)を設定
int UVCCamera::setZoomRel(int zoom)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// ズーム(相対値)の現在値を取得
int UVCCamera::getZoomRel()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// digital multiplier調整
int UVCCamera::updateDigitalMultiplierLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// digital multiplierを設定
int UVCCamera::setDigitalMultiplier(int multiplier)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// digital multiplierの現在値を取得
int UVCCamera::getDigitalMultiplier()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// digital multiplier limit調整
int UVCCamera::updateDigitalMultiplierLimitLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// digital multiplier limitを設定
int UVCCamera::setDigitalMultiplierLimit(int multiplier_limit)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

// digital multiplier limitの現在値を取得
int UVCCamera::getDigitalMultiplierLimit()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// AnalogVideoStandard
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
// AnalogVideoLoackStatus
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
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::setAverageBitrate(int bitrate)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::getAverageBitrate()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// SyncRefFrameStatus
int UVCCamera::updateSyncRefFrameLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::setSyncRefFrame(int value)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::getSyncRefFrame()
{
    ENTER();

    RETURN(0, int);
}

//======================================================================
// CPBSizeStatus
int UVCCamera::updateCPBSizeLimit(int &min, int &max, int &def)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::setCPBSize(int value)
{
    ENTER();
    int ret = UVC_ERROR_IO;

    RETURN(ret, int);
}

int UVCCamera::getCPBSize()
{
    ENTER();

    RETURN(0, int);
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