/*
 * UVCCamera
 * library and sample to access to UVC web camera on non-rooted Android device
 *
 * Copyright (c) 2014-2017 Jerry.Zhang@polycom.com
 *
 * File name: UVCRecord.cpp
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
#pragma implementation "UVCRecord.h"

#include <stdlib.h>
#include <linux/time.h>
#include <unistd.h>

#if 1	// set 1 if you don't need debug log
#ifndef LOG_NDEBUG
#define	LOG_NDEBUG		// w/o LOGV/LOGD/MARK
#endif
#undef USE_LOGALL
#else
#define USE_LOGALL
#undef LOG_NDEBUG
//	#undef NDEBUG
#endif

#include "utilbase.h"
#include "UVCRecord.h"
#include "libuvc_internal.h"

#define	LOCAL_DEBUG 0
#define MAX_FRAME 2
#define FRAME_POOL_SZ MAX_FRAME + 1

UVCRecord::UVCRecord(uvc_device_handle_t *devh)
    : UVCStream(devh),
      requestProfile(DEFAULT_RECORD_PROFILE),
      requestUsage(DEFAULT_RECORD_USAGE),
      recordBytes(DEFAULT_RECORD_WIDTH * DEFAULT_RECORD_HEIGHT * 3 / 4),
      recordFormat(WINDOW_FORMAT_RGBA_8888),
      stream_probed(false),
      stream_committed(false)
{

    ENTER();

    requestWidth = DEFAULT_RECORD_WIDTH;
    requestHeight = DEFAULT_RECORD_HEIGHT;
    requestMinFps = DEFAULT_RECORD_FPS_MIN;
    requestMaxFps = DEFAULT_RECORD_FPS_MAX;
    requestMode = DEFAULT_RECORD_MODE;
    frameWidth = DEFAULT_RECORD_WIDTH;
    frameHeight = DEFAULT_RECORD_HEIGHT;
    frameBytes = DEFAULT_RECORD_WIDTH * DEFAULT_RECORD_HEIGHT * 2; // YUYV
    frameMode = 0;

    memset(&stream_ctrl, 0, sizeof(uvc_stream_ctrl_t));

    EXIT();
}

UVCRecord::~UVCRecord()
{

    ENTER();

    EXIT();
}

int UVCRecord::setRecordSize(int width, int height, int profile, int min_fps, int max_fps, int mode, float bandwidth)
{
    ENTER();

    int result = 0;

    if (mode == DEFAULT_RECORD_MODE) {
        requestWidth = width;
        requestHeight = height;
        requestMinFps = min_fps;
        requestMaxFps = max_fps;
        requestMode = mode;
        requestProfile = profile;
        requestUsage = DEFAULT_RECORD_USAGE;
        requestBandwidth = bandwidth;

        result = uvc_get_stream_ctrl_format_size_fps_profile_usage(mDeviceHandle, &stream_ctrl,
                 UVC_FRAME_FORMAT_H_264,
                 requestWidth, requestHeight,
                 requestProfile, requestUsage,
                 requestMinFps, requestMaxFps);
    } else {
        result = -1;
    }

    if (!result)
        stream_probed = true;

    RETURN(result, int);
}

int UVCRecord::setRecordSize(int width, int height, int profile, int usage, int min_fps, int max_fps, int mode, float bandwidth)
{
    ENTER();

    int result = 0;

    if (mode == DEFAULT_RECORD_MODE) {
        requestWidth = width;
        requestHeight = height;
        requestMinFps = min_fps;
        requestMaxFps = max_fps;
        requestMode = mode;
        requestProfile = profile;
        requestUsage = usage;
        requestBandwidth = bandwidth;

        result = uvc_get_stream_ctrl_format_size_fps_profile_usage(mDeviceHandle, &stream_ctrl,
                 UVC_FRAME_FORMAT_H_264,
                 requestWidth, requestHeight,
                 requestProfile, requestUsage,
                 requestMinFps, requestMaxFps);
    } else {
        result = -1;
    }

    if (!result)
        stream_probed = true;

    RETURN(result, int);
}

int UVCRecord::commitRecordSize(int width, int height, int profile, int usage, int min_fps, int max_fps, int mode, float bandwidth)
{
    ENTER();

    int result = 0;

    if (mode == DEFAULT_RECORD_MODE) {
        requestWidth = width;
        requestHeight = height;
        requestMinFps = min_fps;
        requestMaxFps = max_fps;
        requestMode = mode;
        requestProfile = profile;
        requestUsage = usage;
        requestBandwidth = bandwidth;

        result = uvc_get_and_commit_stream_ctrl_format_size_fps_profile_usage(mDeviceHandle, &stream_ctrl,
                 UVC_FRAME_FORMAT_H_264,
                 requestWidth, requestHeight,
                 requestProfile, requestUsage,
                 requestMinFps, requestMaxFps);
    } else {
        result = -1;
    }

    if (!result)
        stream_probed = stream_committed = true;

    RETURN(result, int);
}

int UVCRecord::setFrameCallback(JNIEnv *env, jobject frame_callback_obj, int pixel_format)
{

    ENTER();

    UVCStream::setFrameCallback(env, frame_callback_obj, 1, "onRecordFrame", "(Ljava/nio/ByteBuffer;)V");

    RETURN(0, int);
}

int UVCRecord::startRecord()
{
    ENTER();

    int result = EXIT_FAILURE;

    result = startStreaming();

    RETURN(result, int);
}

int UVCRecord::stopRecord()
{
    ENTER();

    stopStreaming();

    stream_probed = stream_committed = false;

    RETURN(0, int);
}

int UVCRecord::prepare_streaming(uvc_stream_ctrl_t *ctrl)
{
    uvc_error_t result = UVC_SUCCESS;

    ENTER();

    if (stream_probed) {
        memcpy(ctrl, &stream_ctrl, sizeof(uvc_stream_ctrl_t));
    } else {
        result = uvc_get_stream_ctrl_format_size_fps_profile_usage(mDeviceHandle, ctrl,
                 UVC_FRAME_FORMAT_H_264,
                 requestWidth, requestHeight,
                 requestProfile, requestUsage,
                 requestMinFps, requestMaxFps);
    }

    if (LIKELY(!result)) {
#if LOCAL_DEBUG
        uvc_print_stream_ctrl(ctrl, stderr);
#endif
        uvc_frame_desc_t *frame_desc;
        result = uvc_get_frame_desc(mDeviceHandle, ctrl, &frame_desc);

        if (LIKELY(!result)) {
            frameWidth = frame_desc->wWidth;
            frameHeight = frame_desc->wHeight;
            LOGI("frameSize=(%d,%d)@%s", frameWidth, frameHeight, "H264");
        } else {
            frameWidth = requestWidth;
            frameHeight = requestHeight;
        }

        frameMode = requestMode;
        frameBytes = frameWidth * frameHeight * 3 / 4;

    } else {
        LOGE("could not negotiate with camera:err=%d", result);
    }

    RETURN(result, int);
}

void UVCRecord::do_streaming(uvc_stream_ctrl_t *ctrl)
{
    ENTER();

    uvc_frame_t *frame = NULL;
    uvc_error_t result = uvc_start_streaming_bandwidth_committed(
                             mDeviceHandle,
                             ctrl,
                             uvc_streaming_frame_callback,
                             (void *)this,
                             requestBandwidth,
                             0,
                             stream_committed ? 1 : 0);

    if (LIKELY(!result)) {
        clearStreamingFrame();
        pthread_create(&capture_thread, NULL, capture_thread_func, (void *)this);

#if LOCAL_DEBUG
        LOGI("Recording...");
#endif

        // H264 mode
        for (; LIKELY(isRunning()) ;) {
            frame = waitStreamingFrame();

            if (LIKELY(frame)) {
                addCaptureFrame(frame);
            }
        }

        pthread_cond_signal(&capture_sync);
#if LOCAL_DEBUG
        LOGI("record_thread_func:wait for all callbacks complete");
#endif
        uvc_stop_streaming(mDeviceHandle, ctrl);
#if LOCAL_DEBUG
        LOGI("Streaming finished");
#endif
    } else {
        uvc_perror(result, "failed start_streaming");
    }

    EXIT();
}

/**
 * the actual function for capturing
 */
void UVCRecord::do_capture(JNIEnv *env)
{

    ENTER();

    clearCaptureFrame();

    for (; isRunning() ;) {
        mIsCapturing = true;

        do_capture_idle_loop(env);

        pthread_cond_broadcast(&capture_sync);
    }   // end of for (; isRunning() ;)

    EXIT();
}

void UVCRecord::do_capture_idle_loop(JNIEnv *env)
{
    ENTER();

    for (; isRunning() && isCapturing() ;) {
        do_capture_callback(env, waitCaptureFrame());
    }

    EXIT();
}

/**
* call IFrameCallback#onRecordFrame if needs
 */
void UVCRecord::do_capture_callback(JNIEnv *env, uvc_frame_t *frame)
{
    ENTER();

    if (LIKELY(frame)) {
        uvc_frame_t *callback_frame = frame;

        if (mFrameCallbackObj) {
            jobject buf = env->NewDirectByteBuffer(callback_frame->data, callback_frame->actual_bytes);
            env->CallVoidMethod(mFrameCallbackObj, iframecallback_fields.onRecordFrame, buf);
            env->ExceptionClear();
            env->DeleteLocalRef(buf);
        }

SKIP:
        recycle_frame(callback_frame);
    }

    EXIT();
}
