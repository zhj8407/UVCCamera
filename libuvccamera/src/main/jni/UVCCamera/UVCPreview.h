/*
 * UVCCamera
 * library and sample to access to UVC web camera on non-rooted Android device
 *
 * Copyright (c) 2014-2017 saki t_saki@serenegiant.com
 *
 * File name: UVCPreview.h
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

#ifndef UVCPREVIEW_H_
#define UVCPREVIEW_H_

#include "libUVCCamera.h"
#include <pthread.h>
#include <android/native_window.h>
#include "objectarray.h"

#include "UVCStream.h"

#define DEFAULT_PREVIEW_WIDTH 640
#define DEFAULT_PREVIEW_HEIGHT 480
#define DEFAULT_PREVIEW_FPS_MIN 1
#define DEFAULT_PREVIEW_FPS_MAX 30
#define DEFAULT_PREVIEW_MODE 0

#define PIXEL_FORMAT_RAW 0		// same as PIXEL_FORMAT_YUV
#define PIXEL_FORMAT_YUV 1
#define PIXEL_FORMAT_RGB565 2
#define PIXEL_FORMAT_RGBX 3
#define PIXEL_FORMAT_YUV20SP 4
#define PIXEL_FORMAT_NV21 5		// YVU420SemiPlanar

class UVCPreview : public UVCStream
{
private:
    ANativeWindow *mPreviewWindow;

    int previewFormat;
    size_t previewBytes;
    int mPixelFormat;
//
    ANativeWindow *mCaptureWindow;

    convFunc_t mFrameCallbackFunc;

    size_t callbackPixelBytes;
//
    void clearDisplay();
    virtual int prepare_streaming();
    virtual void do_streaming();
    uvc_frame_t *draw_preview_one(uvc_frame_t *frame, ANativeWindow **window, convFunc_t func, int pixelBytes);
//
    virtual void do_capture(JNIEnv *env);
    void do_capture_surface(JNIEnv *env);
    void do_capture_idle_loop(JNIEnv *env);
    void do_capture_callback(JNIEnv *env, uvc_frame_t *frame);
    void callbackPixelFormatChanged();
public:
    UVCPreview(uvc_device_handle_t *devh, v4l2_dev_t *v4l2Dev);
    virtual ~UVCPreview();

    int setPreviewSize(int width, int height, int min_fps, int max_fps, int mode, float bandwidth = 1.0f);
    int setPreviewDisplay(ANativeWindow *preview_window);
    int setFrameCallback(JNIEnv *env, jobject frame_callback_obj, int pixel_format);
    int startPreview();
    int stopPreview();

    int setCaptureDisplay(ANativeWindow *capture_window);
};

#endif /* UVCPREVIEW_H_ */
