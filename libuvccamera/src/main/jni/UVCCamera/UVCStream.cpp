/*
 * UVCCamera
 * library and sample to access to UVC web camera on non-rooted Android device
 *
 * Copyright (c) 2014-2017 Jerry.Zhang@polycom.com
 *
 * File name: UVCStream.cpp
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
#pragma implementation "UVCStream.h"

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
#include "UVCStream.h"
#include "libuvc_internal.h"

#define	LOCAL_DEBUG 0
#define MAX_FRAME 2
#define FRAME_POOL_SZ MAX_FRAME + 1

UVCStream::UVCStream(uvc_device_handle_t *devh)
    : mDeviceHandle(devh),
      requestBandwidth(DEFAULT_BANDWIDTH),
      mIsRunning(false),
      mIsCapturing(false),
      captureQueu(NULL)
{

    ENTER();
    pthread_cond_init(&streaming_sync, NULL);
    pthread_mutex_init(&streaming_mutex, NULL);

    pthread_cond_init(&capture_sync, NULL);
    pthread_mutex_init(&capture_mutex, NULL);

    pthread_mutex_init(&pool_mutex, NULL);
    EXIT();
}

UVCStream::~UVCStream()
{
    ENTER();

    clearStreamingFrame();
    clearCaptureFrame();

    clear_pool();
    pthread_mutex_destroy(&pool_mutex);

    pthread_mutex_destroy(&streaming_mutex);
    pthread_cond_destroy(&streaming_sync);

    pthread_mutex_destroy(&capture_mutex);
    pthread_cond_destroy(&capture_sync);

    EXIT();
}

/**
 * get uvc_frame_t from frame pool
 * if pool is empty, create new frame
 * this function does not confirm the frame size
 * and you may need to confirm the size
 */
uvc_frame_t* UVCStream::get_frame(size_t data_bytes)
{
    uvc_frame_t *frame = NULL;
    pthread_mutex_lock(&pool_mutex);
    {
        if (!mFramePool.isEmpty()) {
            frame = mFramePool.last();
        }
    }
    pthread_mutex_unlock(&pool_mutex);

    if UNLIKELY(!frame) {
        LOGW("allocate new frame");
        frame = uvc_allocate_frame(data_bytes);
    }

    return frame;
}

void UVCStream::recycle_frame(uvc_frame_t *frame)
{
    pthread_mutex_lock(&pool_mutex);

    if (LIKELY(mFramePool.size() < FRAME_POOL_SZ)) {
        mFramePool.put(frame);
        frame = NULL;
    }

    pthread_mutex_unlock(&pool_mutex);

    if (UNLIKELY(frame)) {
        uvc_free_frame(frame);
    }
}


void UVCStream::init_pool(size_t data_bytes)
{
    ENTER();

    clear_pool();
    pthread_mutex_lock(&pool_mutex);
    {
        for (int i = 0; i < FRAME_POOL_SZ; i++) {
            mFramePool.put(uvc_allocate_frame(data_bytes));
        }
    }
    pthread_mutex_unlock(&pool_mutex);

    EXIT();
}

void UVCStream::clear_pool()
{
    ENTER();

    pthread_mutex_lock(&pool_mutex);
    {
        const int n = mFramePool.size();

        for (int i = 0; i < n; i++) {
            uvc_free_frame(mFramePool[i]);
        }

        mFramePool.clear();
    }
    pthread_mutex_unlock(&pool_mutex);
    EXIT();
}

void UVCStream::addStreamingFrame(uvc_frame_t *frame)
{

    pthread_mutex_lock(&streaming_mutex);

    if (isRunning() && (streamingFrames.size() < MAX_FRAME)) {
        streamingFrames.put(frame);
        frame = NULL;
        pthread_cond_signal(&streaming_sync);
    }

    pthread_mutex_unlock(&streaming_mutex);

    if (frame) {
        recycle_frame(frame);
    }
}

uvc_frame_t *UVCStream::waitStreamingFrame()
{
    uvc_frame_t *frame = NULL;
    pthread_mutex_lock(&streaming_mutex);
    {
        if (!streamingFrames.size()) {
            pthread_cond_wait(&streaming_sync, &streaming_mutex);
        }

        if (LIKELY(isRunning() && streamingFrames.size() > 0)) {
            frame = streamingFrames.remove(0);
        }
    }
    pthread_mutex_unlock(&streaming_mutex);
    return frame;
}

void UVCStream::clearStreamingFrame()
{
    pthread_mutex_lock(&streaming_mutex);
    {
        for (int i = 0; i < streamingFrames.size(); i++)
            recycle_frame(streamingFrames[i]);

        streamingFrames.clear();
    }
    pthread_mutex_unlock(&streaming_mutex);
}

void UVCStream::uvc_streaming_frame_callback(uvc_frame_t *frame, void *vptr_args)
{
    UVCStream *stream = reinterpret_cast<UVCStream *>(vptr_args);

    if UNLIKELY(!stream->isRunning() || !frame || !frame->frame_format || !frame->data || !frame->data_bytes) return;

    if (UNLIKELY(
                ((frame->frame_format != UVC_FRAME_FORMAT_MJPEG) && (frame->actual_bytes < stream->frameBytes))
                || (frame->width != stream->frameWidth) || (frame->height != stream->frameHeight))) {

#if LOCAL_DEBUG
        LOGD("broken frame!:format=%d,actual_bytes=%d/%d(%d,%d/%d,%d)",
             frame->frame_format, frame->actual_bytes, stream->frameBytes,
             frame->width, frame->height, stream->frameWidth, stream->frameHeight);
#endif
        return;
    }

    if (LIKELY(stream->isRunning())) {
        uvc_frame_t *copy = stream->get_frame(frame->data_bytes);

        if (UNLIKELY(!copy)) {
#if LOCAL_DEBUG
            LOGE("uvc_callback:unable to allocate duplicate frame!");
#endif
            return;
        }

        uvc_error_t ret = uvc_duplicate_frame(frame, copy);

        if (UNLIKELY(ret)) {
            stream->recycle_frame(copy);
            return;
        }

        stream->addStreamingFrame(copy);
    }
}

void UVCStream::addCaptureFrame(uvc_frame_t *frame)
{
    pthread_mutex_lock(&capture_mutex);

    if (LIKELY(isRunning())) {
        // keep only latest one
        if (captureQueu) {
            recycle_frame(captureQueu);
        }

        captureQueu = frame;
        pthread_cond_broadcast(&capture_sync);
    }

    pthread_mutex_unlock(&capture_mutex);
}

/**
 * get frame data for capturing, if not exist, block and wait
 */
uvc_frame_t *UVCStream::waitCaptureFrame()
{
    uvc_frame_t *frame = NULL;
    pthread_mutex_lock(&capture_mutex);
    {
        if (!captureQueu) {
            pthread_cond_wait(&capture_sync, &capture_mutex);
        }

        if (LIKELY(isRunning() && captureQueu)) {
            frame = captureQueu;
            captureQueu = NULL;
        }
    }
    pthread_mutex_unlock(&capture_mutex);
    return frame;
}

/**
 * clear drame data for capturing
 */
void UVCStream::clearCaptureFrame()
{
    pthread_mutex_lock(&capture_mutex);
    {
        if (captureQueu)
            recycle_frame(captureQueu);

        captureQueu = NULL;
    }
    pthread_mutex_unlock(&capture_mutex);
}

//======================================================================
/*
 * thread function
 * @param vptr_args pointer to UVCStream instance
 */
// static
void *UVCStream::capture_thread_func(void *vptr_args)
{
    int result;

    ENTER();
    UVCStream *stream = reinterpret_cast<UVCStream *>(vptr_args);

    if (LIKELY(stream)) {
        JavaVM *vm = getVM();
        JNIEnv *env;
        // attach to JavaVM
        vm->AttachCurrentThread(&env, NULL);
        stream->do_capture(env);   // never return until finish previewing
        // detach from JavaVM
        vm->DetachCurrentThread();
        MARK("DetachCurrentThread");
    }

    PRE_EXIT();
    pthread_exit(NULL);
}
