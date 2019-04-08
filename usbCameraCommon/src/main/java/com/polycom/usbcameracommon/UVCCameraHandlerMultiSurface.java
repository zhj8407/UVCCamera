/*
 *  UVCCamera
 *  library and sample to access to UVC web camera on non-rooted Android device
 *
 * Copyright (c) 2014-2017 saki t_saki@serenegiant.com
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 *  All files in the folder are under this Apache License, Version 2.0.
 *  Files in the libjpeg-turbo, libusb, libuvc, rapidjson folder
 *  may have a different license, see the respective files.
 */

package com.polycom.usbcameracommon;

import android.app.Activity;
import android.view.Surface;

import com.polycom.uvccamera.usb.UVCCamera;
import com.polycom.widget.CameraViewInterface;
import com.serenegiant.glutils.RendererHolder;

public class UVCCameraHandlerMultiSurface extends AbstractUVCCameraHandler {
    /**
     * create UVCCameraHandlerMultiSurface, use MediaVideoEncoder, try MJPEG, default bandwidth
     */
    public static final UVCCameraHandlerMultiSurface createHandler(
            final Activity parent, final CameraViewInterface cameraView,
            final int width, final int height) {

        return createHandler(parent, cameraView, 1, width, height, UVCCamera.FRAME_FORMAT_MJPEG,
                UVCCamera.DEFAULT_BANDWIDTH,
                UVCCamera.DEFAULT_RECORD_WIDTH, UVCCamera.DEFAULT_RECORD_HEIGHT,
                UVCCamera.DEFAULT_RECORD_MODE, UVCCamera.DEFAULT_RECORD_PROFILE,
                UVCCamera.H264_USAGE_1,
                UVCCamera.DEFAULT_BANDWIDTH);
    }

    /**
     * create UVCCameraHandlerMultiSurface, use MediaVideoEncoder, try MJPEG
     */
    public static final UVCCameraHandlerMultiSurface createHandler(
            final Activity parent, final CameraViewInterface cameraView,
            final int width, final int height, final float bandwidthFactor) {

        return createHandler(parent, cameraView, 1, width, height, UVCCamera.FRAME_FORMAT_MJPEG,
                UVCCamera.DEFAULT_BANDWIDTH,
                UVCCamera.DEFAULT_RECORD_WIDTH, UVCCamera.DEFAULT_RECORD_HEIGHT,
                UVCCamera.DEFAULT_RECORD_MODE, UVCCamera.DEFAULT_RECORD_PROFILE,
                UVCCamera.H264_USAGE_1,
                UVCCamera.DEFAULT_BANDWIDTH);
    }

    /**
     * create UVCCameraHandlerMultiSurface, try MJPEG, default bandwidth
     */
    public static final UVCCameraHandlerMultiSurface createHandler(
            final Activity parent, final CameraViewInterface cameraView,
            final int encoderType, final int width, final int height) {

        return createHandler(parent, cameraView, encoderType, width, height,
                UVCCamera.FRAME_FORMAT_MJPEG, UVCCamera.DEFAULT_BANDWIDTH,
                UVCCamera.DEFAULT_RECORD_WIDTH, UVCCamera.DEFAULT_RECORD_HEIGHT,
                UVCCamera.DEFAULT_RECORD_MODE, UVCCamera.DEFAULT_RECORD_PROFILE,
                UVCCamera.H264_USAGE_1,
                UVCCamera.DEFAULT_BANDWIDTH);
    }

    /**
     * create UVCCameraHandlerMultiSurface, default bandwidth
     */
    public static final UVCCameraHandlerMultiSurface createHandler(
            final Activity parent, final CameraViewInterface cameraView,
            final int encoderType, final int width, final int height, final int format) {

        return createHandler(parent, cameraView, encoderType, width, height, format,
                UVCCamera.DEFAULT_BANDWIDTH,
                UVCCamera.DEFAULT_RECORD_WIDTH, UVCCamera.DEFAULT_RECORD_HEIGHT,
                UVCCamera.DEFAULT_RECORD_MODE, UVCCamera.DEFAULT_RECORD_PROFILE,
                UVCCamera.H264_USAGE_1,
                UVCCamera.DEFAULT_BANDWIDTH);
    }

    /**
     * create UVCCameraHandlerMultiSurface
     *
     * @param encoderType   0: use MediaSurfaceEncoder, 1: use MediaVideoEncoder, 2: use
     *                      MediaVideoBufferEncoder
     * @param previewFormat either UVCCamera.FRAME_FORMAT_YUYV(0) or UVCCamera
     *                         .FRAME_FORMAT_MJPEG(1)
     */
    public static final UVCCameraHandlerMultiSurface createHandler(
            final Activity parent, final CameraViewInterface cameraView,
            final int encoderType, final int previewWidth, final int previewHeight,
            final int previewFormat,
            final float previewBandwidthFactor,
            int recordWidth, int recordHeight, int recordFormat,
            int recordProfile, int recordUsage,
            float recordBandwidthFactor) {

        final CameraThread thread = new CameraThread(UVCCameraHandler.class, parent, cameraView,
                encoderType, previewWidth, previewHeight, previewFormat, previewBandwidthFactor,
                recordWidth, recordHeight, recordFormat, recordProfile, recordUsage,
                recordBandwidthFactor);
        thread.start();
        return (UVCCameraHandlerMultiSurface) thread.getHandler();
    }

    private RendererHolder mRendererHolder;

    protected UVCCameraHandlerMultiSurface(final CameraThread thread) {
        super(thread);
        mRendererHolder = new RendererHolder(thread.getPreviewWidth(), thread.getPreviewHeight(),
                null);
    }

    public synchronized void release() {
        if (mRendererHolder != null) {
            mRendererHolder.release();
            mRendererHolder = null;
        }
        super.release();
    }

    public synchronized void resize(final int width, final int height) {
        super.resize(width, height);
        if (mRendererHolder != null) {
            mRendererHolder.resize(width, height);
        }
    }

    public synchronized void startPreview() {
        checkReleased();
        if (mRendererHolder != null) {
            super.startPreview(mRendererHolder.getSurface());
        } else {
            throw new IllegalStateException();
        }
    }

    public synchronized void addSurface(final int surfaceId, final Surface surface,
            final boolean isRecordable) {
        checkReleased();
        mRendererHolder.addSurface(surfaceId, surface, isRecordable);
    }

    public synchronized void removeSurface(final int surfaceId) {
        if (mRendererHolder != null) {
            mRendererHolder.removeSurface(surfaceId);
        }
    }

    @Override
    public void captureStill() {
        checkReleased();
        super.captureStill();
    }

    @Override
    public void captureStill(final String path) {
        checkReleased();
        post(new Runnable() {
            @Override
            public void run() {
                synchronized (UVCCameraHandlerMultiSurface.this) {
                    if (mRendererHolder != null) {
                        mRendererHolder.captureStill(path);
                        updateMedia(path);
                    }
                }
            }
        });
    }
}
