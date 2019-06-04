/*
 * UVCCamera
 * library and sample to access to UVC web camera on non-rooted Android device
 *
 * Copyright (c) 2014-2017 Jerry.Zhang@polycom.com
 *
 * File name: UVCStream.h
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

#ifndef UVCStream_H_
#define UVCStream_H_

#include <pthread.h>
#include <android/native_window.h>

#include <algorithm>
#include <deque>

#include "uvc_dev.h"
#include "libUVCCamera.h"

#define DEFAULT_PREVIEW_WIDTH 640
#define DEFAULT_PREVIEW_HEIGHT 480
#define DEFAULT_PREVIEW_FPS_MIN 1
#define DEFAULT_PREVIEW_FPS_MAX 30
#define DEFAULT_PREVIEW_MODE 0

#define YUYV_FORMAT_PREVIEW_MODE 0
#define MJPG_FORMAT_PREVIEW_MODE 1
#define NV12_FORMAT_PREVIEW_MODE 2
#define H264_FORMAT_RECORD_MODE  3
#define S264_FORMAT_RECORD_MODE  4

#define PIXEL_FORMAT_RAW 0      // same as PIXEL_FORMAT_YUV
#define PIXEL_FORMAT_YUV 1
#define PIXEL_FORMAT_RGB565 2
#define PIXEL_FORMAT_RGBX 3
#define PIXEL_FORMAT_YUV20SP 4
#define PIXEL_FORMAT_NV21 5     // YVU420SemiPlanar
#define PIXEL_FORMAT_H264 6

#define H264_PROFILE_CONSTRAINED_BASELINE 16960
#define H264_PROFILE_HIGH 25600
#define H264_PROFILE_CONSTRAINED_HIGH 25612

#define H264_USAGE_1 1
#define H264_USAGE_2 2

#define DEFAULT_RECORD_WIDTH 1920
#define DEFAULT_RECORD_HEIGHT 1080
#define DEFAULT_RECORD_FPS_MIN 1
#define DEFAULT_RECORD_FPS_MAX 30
#define DEFAULT_RECORD_MODE H264_FORMAT_RECORD_MODE
#define DEFAULT_RECORD_PROFILE H264_PROFILE_CONSTRAINED_BASELINE
#define DEFAULT_RECORD_USAGE H264_USAGE_1

#define DEFAULT_BANDWIDTH 1.0f

#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof((arr)[0]))

typedef uvc_error_t (*convFunc_t)(uvc_frame_t *in, uvc_frame_t *out);

// for callback to Java object
typedef struct
{
    jmethodID onFrame;
    jmethodID onRecordFrame;
} Fields_iframecallback;

const int pixel_formats[] =
{
    V4L2_PIX_FMT_YUYV,
    V4L2_PIX_FMT_MJPEG,
    V4L2_PIX_FMT_NV12,
    V4L2_PIX_FMT_H264,
    V4L2_PIX_FMT_H264_SIMULCAST
};

class UVCStream
{
protected:
    v4l2_dev_t *mV4l2Dev;

    int requestWidth, requestHeight, requestMode;
    int requestMinFps, requestMaxFps;
    float requestBandwidth;
    int frameWidth, frameHeight;
    int frameMode;
    size_t frameBytes;
    //
    volatile bool mIsRunning;
    pthread_t streaming_thread;
    pthread_mutex_t streaming_mutex;
    pthread_cond_t streaming_sync;
    std::deque<uvc_frame_t *> mStreamingFrames;

    static void *streaming_thread_func(void *vptr_args);
    virtual int prepare_streaming() = 0;
    virtual void do_streaming() = 0;

    int startStreaming();
    int stopStreaming();

    // improve performance by reducing memory allocation
    pthread_mutex_t pool_mutex;
    std::vector<uvc_frame_t *> mFramePool;
    uvc_frame_t *get_frame(size_t data_bytes);
    void recycle_frame(uvc_frame_t *frame);
    void init_pool(size_t data_bytes);
    void clear_pool();
    //
    static void uvc_streaming_frame_callback(uvc_frame_t *frame, void *vptr_args);

    void addStreamingFrame(uvc_frame_t *frame);
    uvc_frame_t *waitStreamingFrame();
    void clearStreamingFrame();

    volatile bool mIsCapturing;
    pthread_t capture_thread;
    pthread_mutex_t capture_mutex;
    pthread_cond_t capture_sync;
    uvc_frame_t *captureQueu; // keep latest frame

    static void *capture_thread_func(void *vptr_args);
    //
    void addCaptureFrame(uvc_frame_t *frame);
    uvc_frame_t *waitCaptureFrame();
    void clearCaptureFrame();
    virtual void do_capture(JNIEnv *env) = 0;
    //
    jobject mFrameCallbackObj;
    Fields_iframecallback iframecallback_fields;
    int setFrameCallback(JNIEnv *env, jobject frame_callback_obj, int callback_idx,
                         const char *callback_name, const char *callback_sig);

    inline const bool isRunning() const
    {
        return mIsRunning;
    }

    inline const bool isCapturing() const
    {
        return mIsCapturing;
    }

public:
    UVCStream(v4l2_dev_t *v4l2Dev);
    virtual ~UVCStream();
};

#endif /* UVCStream_H_ */
