/*
 * UVCCamera
 * library and sample to access to UVC web camera on non-rooted Android device
 *
 * Copyright (c) 2014-2017 saki t_saki@serenegiant.com
 *
 * File name: UVCPreview.cpp
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
#pragma implementation "UVCPreview.h"

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
#include "UVCPreview.h"
#include "libuvc_internal.h"

#include "v4l2_core.h"

#define	LOCAL_DEBUG 0
#define MAX_FRAME 2
#define PREVIEW_PIXEL_BYTES 4	// RGBA/RGBX
#define FRAME_POOL_SZ MAX_FRAME + 1

UVCPreview::UVCPreview(uvc_device_handle_t *devh, v4l2_dev_t *v4l2Dev)
    : UVCStream(devh, v4l2Dev),
      mPreviewWindow(NULL),
      mCaptureWindow(NULL),
      previewBytes(DEFAULT_PREVIEW_WIDTH * DEFAULT_PREVIEW_HEIGHT * PREVIEW_PIXEL_BYTES),
      previewFormat(WINDOW_FORMAT_RGBA_8888),
      mFrameCallbackFunc(NULL),
      callbackPixelBytes(2)
{

    ENTER();

    requestWidth = DEFAULT_PREVIEW_WIDTH;
    requestHeight = DEFAULT_PREVIEW_HEIGHT;
    requestMinFps = DEFAULT_PREVIEW_FPS_MIN;
    requestMaxFps = DEFAULT_PREVIEW_FPS_MAX;
    requestMode = DEFAULT_PREVIEW_MODE;
    frameWidth = DEFAULT_PREVIEW_WIDTH;
    frameHeight = DEFAULT_PREVIEW_HEIGHT;
    frameBytes = DEFAULT_PREVIEW_WIDTH * DEFAULT_PREVIEW_HEIGHT * 2; // YUYV
    frameMode = 0;

    EXIT();
}

UVCPreview::~UVCPreview()
{

    ENTER();

    if (mPreviewWindow)
        ANativeWindow_release(mPreviewWindow);

    mPreviewWindow = NULL;

    if (mCaptureWindow)
        ANativeWindow_release(mCaptureWindow);

    mCaptureWindow = NULL;

    EXIT();
}

int UVCPreview::setPreviewSize(int width, int height, int min_fps, int max_fps, int mode, float bandwidth)
{
    ENTER();

    int result = 0;

    if ((requestWidth != width) || (requestHeight != height) || (requestMode != mode)) {
        requestWidth = width;
        requestHeight = height;
        requestMinFps = min_fps;
        requestMaxFps = max_fps;
        requestMode = mode;
        requestBandwidth = bandwidth;

        v4l2core_prepare_new_format(mV4l2Dev, !requestMode ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_MJPEG);
        v4l2core_prepare_new_resolution(mV4l2Dev, requestWidth, requestHeight, 0, 0, 0, 0, 0, 0);
        v4l2core_define_fps(mV4l2Dev, 1, 30);
    }

    RETURN(result, int);
}

int UVCPreview::setPreviewDisplay(ANativeWindow *preview_window)
{
    ENTER();
    pthread_mutex_lock(&streaming_mutex);
    {
        if (mPreviewWindow != preview_window) {
            if (mPreviewWindow)
                ANativeWindow_release(mPreviewWindow);

            mPreviewWindow = preview_window;

            if (LIKELY(mPreviewWindow)) {
                ANativeWindow_setBuffersGeometry(mPreviewWindow,
                                                 frameWidth, frameHeight, previewFormat);
            }
        }
    }
    pthread_mutex_unlock(&streaming_mutex);
    RETURN(0, int);
}

int UVCPreview::setFrameCallback(JNIEnv *env, jobject frame_callback_obj, int pixel_format)
{

    ENTER();

    UVCStream::setFrameCallback(env, frame_callback_obj, 0, "onFrame", "(Ljava/nio/ByteBuffer;)V");

    pthread_mutex_lock(&capture_mutex);
    {
        if (frame_callback_obj) {
            mPixelFormat = pixel_format;
            callbackPixelFormatChanged();
        }
    }
    pthread_mutex_unlock(&capture_mutex);
    RETURN(0, int);
}

void UVCPreview::callbackPixelFormatChanged()
{
    mFrameCallbackFunc = NULL;
    const size_t sz = requestWidth * requestHeight;

    switch (mPixelFormat) {
        case PIXEL_FORMAT_RAW:
            LOGI("PIXEL_FORMAT_RAW:");
            callbackPixelBytes = sz * 2;
            break;

        case PIXEL_FORMAT_YUV:
            LOGI("PIXEL_FORMAT_YUV:");
            callbackPixelBytes = sz * 2;
            break;

        case PIXEL_FORMAT_RGB565:
            LOGI("PIXEL_FORMAT_RGB565:");
            mFrameCallbackFunc = uvc_any2rgb565;
            callbackPixelBytes = sz * 2;
            break;

        case PIXEL_FORMAT_RGBX:
            LOGI("PIXEL_FORMAT_RGBX:");
            mFrameCallbackFunc = uvc_any2rgbx;
            callbackPixelBytes = sz * 4;
            break;

        case PIXEL_FORMAT_YUV20SP:
            LOGI("PIXEL_FORMAT_YUV20SP:");
            mFrameCallbackFunc = uvc_yuyv2iyuv420SP;
            callbackPixelBytes = (sz * 3) / 2;
            break;

        case PIXEL_FORMAT_NV21:
            LOGI("PIXEL_FORMAT_NV21:");
            mFrameCallbackFunc = uvc_yuyv2yuv420SP;
            callbackPixelBytes = (sz * 3) / 2;
            break;
    }
}

void UVCPreview::clearDisplay()
{
    ENTER();

    ANativeWindow_Buffer buffer;
    pthread_mutex_lock(&capture_mutex);
    {
        if (LIKELY(mCaptureWindow)) {
            if (LIKELY(ANativeWindow_lock(mCaptureWindow, &buffer, NULL) == 0)) {
                uint8_t *dest = (uint8_t *)buffer.bits;
                const size_t bytes = buffer.width * PREVIEW_PIXEL_BYTES;
                const int stride = buffer.stride * PREVIEW_PIXEL_BYTES;

                for (int i = 0; i < buffer.height; i++) {
                    memset(dest, 0, bytes);
                    dest += stride;
                }

                ANativeWindow_unlockAndPost(mCaptureWindow);
            }
        }
    }
    pthread_mutex_unlock(&capture_mutex);
    pthread_mutex_lock(&streaming_mutex);
    {
        if (LIKELY(mPreviewWindow)) {
            if (LIKELY(ANativeWindow_lock(mPreviewWindow, &buffer, NULL) == 0)) {
                uint8_t *dest = (uint8_t *)buffer.bits;
                const size_t bytes = buffer.width * PREVIEW_PIXEL_BYTES;
                const int stride = buffer.stride * PREVIEW_PIXEL_BYTES;

                for (int i = 0; i < buffer.height; i++) {
                    memset(dest, 0, bytes);
                    dest += stride;
                }

                ANativeWindow_unlockAndPost(mPreviewWindow);
            }
        }
    }
    pthread_mutex_unlock(&streaming_mutex);

    EXIT();
}

int UVCPreview::startPreview()
{
    ENTER();

    int result = EXIT_FAILURE;

    pthread_mutex_lock(&streaming_mutex);
    if (LIKELY(mPreviewWindow)) {
        pthread_mutex_unlock(&streaming_mutex);
        result = startStreaming();
    } else {
        pthread_mutex_unlock(&streaming_mutex);
    }

    RETURN(result, int);
}

int UVCPreview::stopPreview()
{
    ENTER();
    bool b = isRunning();

    stopStreaming();

    if (LIKELY(b)) {
        clearDisplay();
    }

    pthread_mutex_lock(&streaming_mutex);

    if (mPreviewWindow) {
        ANativeWindow_release(mPreviewWindow);
        mPreviewWindow = NULL;
    }

    pthread_mutex_unlock(&streaming_mutex);
    pthread_mutex_lock(&capture_mutex);

    if (mCaptureWindow) {
        ANativeWindow_release(mCaptureWindow);
        mCaptureWindow = NULL;
    }

    pthread_mutex_unlock(&capture_mutex);
    RETURN(0, int);
}

//**********************************************************************
//
//**********************************************************************

int UVCPreview::prepare_streaming()
{
    int ret;

    ENTER();

    v4l2core_prepare_new_format(mV4l2Dev, !requestMode ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_MJPEG);
    v4l2core_prepare_new_resolution(mV4l2Dev, requestWidth, requestHeight, 0, 0, 0, 0, 0, 0);
    v4l2core_define_fps(mV4l2Dev, 1, 30);

    ret = v4l2core_update_current_format(mV4l2Dev);

    if (LIKELY(!ret)) {
        frameWidth = requestWidth;
        frameHeight = requestHeight;
        LOGI("frameSize=(%d,%d)@%s", frameWidth, frameHeight, (!requestMode ? "YUYV" : "MJPEG"));
        pthread_mutex_lock(&streaming_mutex);

        if (LIKELY(mPreviewWindow)) {
            ANativeWindow_setBuffersGeometry(mPreviewWindow,
                                                frameWidth, frameHeight, previewFormat);
        }

        pthread_mutex_unlock(&streaming_mutex);

        frameMode = requestMode;
        frameBytes = frameWidth * frameHeight * (!requestMode ? 2 : 4);
        previewBytes = frameWidth * frameHeight * PREVIEW_PIXEL_BYTES;
    } else {
        LOGE("could not negotiate with camera:err=%d", ret);
    }

    RETURN(ret, int);
}

void UVCPreview::do_streaming()
{
    ENTER();

    uvc_frame_t *frame = NULL;
    uvc_frame_t *frame_mjpeg = NULL;
    uvc_error_t result = UVC_SUCCESS;

    v4l2core_set_frame_callback(mV4l2Dev, uvc_streaming_frame_callback, (void *)this);

    int ret = v4l2core_start_stream(mV4l2Dev);

    if (LIKELY(!ret))
    {
        clearStreamingFrame();
        pthread_create(&capture_thread, NULL, capture_thread_func, (void *)this);

#if LOCAL_DEBUG
        LOGI("Streaming...");
#endif

        if (frameMode) {
            // MJPEG mode
            for (; LIKELY(isRunning()) ;) {
                frame_mjpeg = waitStreamingFrame();

                if (LIKELY(frame_mjpeg)) {
                    frame = get_frame(frame_mjpeg->width * frame_mjpeg->height * 2);
                    result = uvc_mjpeg2yuyv(frame_mjpeg, frame);   // MJPEG => yuyv
                    recycle_frame(frame_mjpeg);

                    if (LIKELY(!result)) {
                        frame = draw_preview_one(frame, &mPreviewWindow, uvc_any2rgbx, 4);
                        addCaptureFrame(frame);
                    } else {
                        recycle_frame(frame);
                    }
                }
            }
        } else {
            // yuvyv mode
            for (; LIKELY(isRunning()) ;) {
                frame = waitStreamingFrame();

                if (LIKELY(frame)) {
                    frame = draw_preview_one(frame, &mPreviewWindow, uvc_any2rgbx, 4);
                    addCaptureFrame(frame);
                }
            }
        }

        pthread_cond_signal(&capture_sync);
#if LOCAL_DEBUG
        LOGI("preview_thread_func:wait for all callbacks complete");
#endif
        v4l2core_stop_stream(mV4l2Dev);
#if LOCAL_DEBUG
        LOGI("Streaming finished");
#endif
    } else {
        LOGE("Failed start_streaming: %d\n", ret);
    }

    EXIT();
}

static void copyFrame(const uint8_t *src, uint8_t *dest, const int width, int height, const int stride_src, const int stride_dest)
{
    const int h8 = height % 8;

    for (int i = 0; i < h8; i++) {
        memcpy(dest, src, width);
        dest += stride_dest;
        src += stride_src;
    }

    for (int i = 0; i < height; i += 8) {
        memcpy(dest, src, width);
        dest += stride_dest;
        src += stride_src;
        memcpy(dest, src, width);
        dest += stride_dest;
        src += stride_src;
        memcpy(dest, src, width);
        dest += stride_dest;
        src += stride_src;
        memcpy(dest, src, width);
        dest += stride_dest;
        src += stride_src;
        memcpy(dest, src, width);
        dest += stride_dest;
        src += stride_src;
        memcpy(dest, src, width);
        dest += stride_dest;
        src += stride_src;
        memcpy(dest, src, width);
        dest += stride_dest;
        src += stride_src;
        memcpy(dest, src, width);
        dest += stride_dest;
        src += stride_src;
    }
}


// transfer specific frame data to the Surface(ANativeWindow)
int copyToSurface(uvc_frame_t *frame, ANativeWindow **window)
{
    // ENTER();
    int result = 0;

    if (LIKELY(*window)) {
        ANativeWindow_Buffer buffer;

        if (LIKELY(ANativeWindow_lock(*window, &buffer, NULL) == 0)) {
            // source = frame data
            const uint8_t *src = (uint8_t *)frame->data;
            const int src_w = frame->width * PREVIEW_PIXEL_BYTES;
            const int src_step = frame->width * PREVIEW_PIXEL_BYTES;
            // destination = Surface(ANativeWindow)
            uint8_t *dest = (uint8_t *)buffer.bits;
            const int dest_w = buffer.width * PREVIEW_PIXEL_BYTES;
            const int dest_step = buffer.stride * PREVIEW_PIXEL_BYTES;
            // use lower transfer bytes
            const int w = src_w < dest_w ? src_w : dest_w;
            // use lower height
            const int h = frame->height < buffer.height ? frame->height : buffer.height;
            // transfer from frame data to the Surface
            copyFrame(src, dest, w, h, src_step, dest_step);
            ANativeWindow_unlockAndPost(*window);
        } else {
            result = -1;
        }
    } else {
        result = -1;
    }

    return result; //RETURN(result, int);
}

// changed to return original frame instead of returning converted frame even if convert_func is not null.
uvc_frame_t *UVCPreview::draw_preview_one(uvc_frame_t *frame, ANativeWindow **window, convFunc_t convert_func, int pixcelBytes)
{
    // ENTER();

    int b = 0;
    pthread_mutex_lock(&streaming_mutex);
    {
        b = *window != NULL;
    }
    pthread_mutex_unlock(&streaming_mutex);

    if (LIKELY(b)) {
        uvc_frame_t *converted;

        if (convert_func) {
            converted = get_frame(frame->width * frame->height * pixcelBytes);

            if LIKELY(converted) {
                b = convert_func(frame, converted);

                if (!b) {
                    pthread_mutex_lock(&streaming_mutex);
                    copyToSurface(converted, window);
                    pthread_mutex_unlock(&streaming_mutex);
                } else {
                    LOGE("failed converting");
                }

                recycle_frame(converted);
            }
        } else {
            pthread_mutex_lock(&streaming_mutex);
            copyToSurface(frame, window);
            pthread_mutex_unlock(&streaming_mutex);
        }
    }

    return frame; //RETURN(frame, uvc_frame_t *);
}

//======================================================================
//
//======================================================================

int UVCPreview::setCaptureDisplay(ANativeWindow *capture_window)
{
    ENTER();
    pthread_mutex_lock(&capture_mutex);
    {
        if (isRunning() && isCapturing()) {
            mIsCapturing = false;

            if (mCaptureWindow) {
                pthread_cond_signal(&capture_sync);
                pthread_cond_wait(&capture_sync, &capture_mutex);	// wait finishing capturing
            }
        }

        if (mCaptureWindow != capture_window) {
            // release current Surface if already assigned.
            if (UNLIKELY(mCaptureWindow))
                ANativeWindow_release(mCaptureWindow);

            mCaptureWindow = capture_window;

            // if you use Surface came from MediaCodec#createInputSurface
            // you could not change window format at least when you use
            // ANativeWindow_lock / ANativeWindow_unlockAndPost
            // to write frame data to the Surface...
            // So we need check here.
            if (mCaptureWindow) {
                int32_t window_format = ANativeWindow_getFormat(mCaptureWindow);

                if ((window_format != WINDOW_FORMAT_RGB_565)
                        && (previewFormat == WINDOW_FORMAT_RGB_565)) {
                    LOGE("window format mismatch, cancelled movie capturing.");
                    ANativeWindow_release(mCaptureWindow);
                    mCaptureWindow = NULL;
                }
            }
        }
    }
    pthread_mutex_unlock(&capture_mutex);
    RETURN(0, int);
}

/**
 * the actual function for capturing
 */
void UVCPreview::do_capture(JNIEnv *env)
{

    ENTER();

    clearCaptureFrame();
    callbackPixelFormatChanged();

    for (; isRunning() ;) {
        mIsCapturing = true;

        if (mCaptureWindow) {
            do_capture_surface(env);
        } else {
            do_capture_idle_loop(env);
        }

        pthread_cond_broadcast(&capture_sync);
    }	// end of for (; isRunning() ;)

    EXIT();
}

void UVCPreview::do_capture_idle_loop(JNIEnv *env)
{
    ENTER();

    for (; isRunning() && isCapturing() ;) {
        do_capture_callback(env, waitCaptureFrame());
    }

    EXIT();
}

/**
 * write frame data to Surface for capturing
 */
void UVCPreview::do_capture_surface(JNIEnv *env)
{
    ENTER();

    uvc_frame_t *frame = NULL;
    uvc_frame_t *converted = NULL;
    char *local_picture_path;

    for (; isRunning() && isCapturing() ;) {
        frame = waitCaptureFrame();

        if (LIKELY(frame)) {
            // frame data is always YUYV format.
            if LIKELY(isCapturing()) {
                if (UNLIKELY(!converted)) {
                    converted = get_frame(previewBytes);
                }

                if (LIKELY(converted)) {
                    int b = uvc_any2rgbx(frame, converted);

                    if (!b) {
                        if (LIKELY(mCaptureWindow)) {
                            copyToSurface(converted, &mCaptureWindow);
                        }
                    }
                }
            }

            do_capture_callback(env, frame);
        }
    }

    if (converted) {
        recycle_frame(converted);
    }

    if (mCaptureWindow) {
        ANativeWindow_release(mCaptureWindow);
        mCaptureWindow = NULL;
    }

    EXIT();
}

/**
* call IFrameCallback#onFrame if needs
 */
void UVCPreview::do_capture_callback(JNIEnv *env, uvc_frame_t *frame)
{
    ENTER();

    if (LIKELY(frame)) {
        uvc_frame_t *callback_frame = frame;

        if (mFrameCallbackObj) {
            if (mFrameCallbackFunc) {
                callback_frame = get_frame(callbackPixelBytes);

                if (LIKELY(callback_frame)) {
                    int b = mFrameCallbackFunc(frame, callback_frame);
                    recycle_frame(frame);

                    if (UNLIKELY(b)) {
                        LOGW("failed to convert for callback frame");
                        goto SKIP;
                    }
                } else {
                    LOGW("failed to allocate for callback frame");
                    callback_frame = frame;
                    goto SKIP;
                }
            }

            jobject buf = env->NewDirectByteBuffer(callback_frame->data, callbackPixelBytes);
            env->CallVoidMethod(mFrameCallbackObj, iframecallback_fields.onFrame, buf);
            env->ExceptionClear();
            env->DeleteLocalRef(buf);
        }

SKIP:
        recycle_frame(callback_frame);
    }

    EXIT();
}
