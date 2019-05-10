/*
 * UVCCamera
 * library and sample to access to UVC web camera on non-rooted Android device
 *
 * Copyright (c) 2014-2017 Jerry.Zhang@polycom.com
 *
 * File name: UVCRecord.h
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

#ifndef UVCRECORD_H_
#define UVCRECORD_H_

#include "libUVCCamera.h"
#include <pthread.h>
#include <android/native_window.h>
#include "objectarray.h"

#include "UVCStream.h"

#define H264_PROFILE_CONSTRAINED_BASELINE 16960
#define H264_PROFILE_HIGH 25600
#define H264_PROFILE_CONSTRAINED_HIGH 25612

#define H264_USAGE_1 1
#define H264_USAGE_2 2

#define H264_FORMAT_RECORD_MODE 3
#define S264_FORMAT_RECORD_MODE 4

#define DEFAULT_RECORD_WIDTH 1920
#define DEFAULT_RECORD_HEIGHT 1080
#define DEFAULT_RECORD_FPS_MIN 1
#define DEFAULT_RECORD_FPS_MAX 30
#define DEFAULT_RECORD_MODE H264_FORMAT_RECORD_MODE
#define DEFAULT_RECORD_PROFILE H264_PROFILE_CONSTRAINED_BASELINE
#define DEFAULT_RECORD_USAGE H264_USAGE_1

class UVCRecord : public UVCStream
{
private:
    int requestProfile;
    int frameProfile;
    int requestUsage;

    int recordFormat;
    size_t recordBytes;

    bool stream_probed;
    bool stream_committed;
    //
    virtual int prepare_streaming();
    virtual void do_streaming();

    virtual void do_capture(JNIEnv *env);
    void do_capture_idle_loop(JNIEnv *env);
    void do_capture_callback(JNIEnv *env, uvc_frame_t *frame);

public:
    UVCRecord(v4l2_dev_t *v4l2Dev);
    virtual ~UVCRecord();

    int setRecordSize(int width, int height, int profile, int min_fps, int max_fps, int mode, float bandwidth = 1.0f);
    int setRecordSize(int width, int height, int profile, int usage, int min_fps, int max_fps, int mode, float bandwidth = 1.0f);
    int commitRecordSize(int width, int height, int profile, int usage, int min_fps, int max_fps, int mode, float bandwidth = 1.0f);
    int setFrameCallback(JNIEnv *env, jobject frame_callback_obj, int pixel_format);
    int startRecord();
    int stopRecord();
};

#endif /* UVCRECORD_H_ */
