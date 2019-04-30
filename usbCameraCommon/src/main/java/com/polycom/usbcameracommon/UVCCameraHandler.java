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

import com.polycom.widget.CameraViewInterface;
import com.polycom.uvccamera.usb.UVCCamera;

public class UVCCameraHandler extends AbstractUVCCameraHandler {

    /**
     * create UVCCameraHandler, use MediaVideoEncoder, try MJPEG, default bandwidth
     */
    public static final UVCCameraHandler createHandler(
            final Activity parent, final CameraViewInterface cameraView,
            final int width, final int height) {

        return createHandler(parent, cameraView, 1, width, height, UVCCamera.FRAME_FORMAT_MJPEG,
                UVCCamera.DEFAULT_BANDWIDTH,
                UVCCamera.DEFAULT_RECORD_WIDTH, UVCCamera.DEFAULT_RECORD_HEIGHT,
				UVCCamera.DEFAULT_RECORD_MODE,  UVCCamera.H264_PROFILE_DEFAULT,
				UVCCamera.H264_USAGE_1, UVCCamera.DEFAULT_BANDWIDTH);
    }

    /**
     * create UVCCameraHandler, use MediaVideoEncoder, try MJPEG
     */
    public static final UVCCameraHandler createHandler(
            final Activity parent, final CameraViewInterface cameraView,
            final int width, final int height, final float bandwidthFactor) {

        return createHandler(parent, cameraView, 1, width, height, UVCCamera.FRAME_FORMAT_MJPEG,
                bandwidthFactor,
                UVCCamera.DEFAULT_RECORD_WIDTH, UVCCamera.DEFAULT_RECORD_HEIGHT,
				UVCCamera.DEFAULT_RECORD_MODE, UVCCamera.H264_PROFILE_DEFAULT,
				UVCCamera.H264_USAGE_1, UVCCamera.DEFAULT_BANDWIDTH);

    }

    /**
     * create UVCCameraHandler, try MJPEG, default bandwidth
     *
     * @param encoderType 0: use MediaSurfaceEncoder, 1: use MediaVideoEncoder, 2: use
     *                    MediaVideoBufferEncoder
     */
    public static final UVCCameraHandler createHandler(
            final Activity parent, final CameraViewInterface cameraView,
            final int encoderType, final int width, final int height) {

        return createHandler(parent, cameraView, encoderType, width, height,
                UVCCamera.FRAME_FORMAT_MJPEG, UVCCamera.DEFAULT_BANDWIDTH,
                UVCCamera.DEFAULT_RECORD_WIDTH, UVCCamera.DEFAULT_RECORD_HEIGHT,
                UVCCamera.DEFAULT_RECORD_MODE, UVCCamera.H264_PROFILE_DEFAULT,
                UVCCamera.H264_USAGE_1, UVCCamera.DEFAULT_BANDWIDTH);
    }

    /**
     * create UVCCameraHandler, default bandwidth
     *
     * @param encoderType 0: use MediaSurfaceEncoder, 1: use MediaVideoEncoder, 2: use
     *                    MediaVideoBufferEncoder
     * @param format      either UVCCamera.FRAME_FORMAT_YUYV(0) or UVCCamera.FRAME_FORMAT_MJPEG(1)
     */
    public static final UVCCameraHandler createHandler(
            final Activity parent, final CameraViewInterface cameraView,
            final int encoderType, final int width, final int height, final int format) {

        return createHandler(parent, cameraView, encoderType, width, height, format,
                UVCCamera.DEFAULT_BANDWIDTH,
                UVCCamera.DEFAULT_RECORD_WIDTH, UVCCamera.DEFAULT_RECORD_HEIGHT,
                UVCCamera.DEFAULT_RECORD_MODE, UVCCamera.H264_PROFILE_DEFAULT,
				UVCCamera.H264_USAGE_1, UVCCamera.DEFAULT_BANDWIDTH);
    }

    /**
     * create UVCCameraHandler
     *
     * @param encoderType   0: use MediaSurfaceEncoder, 1: use MediaVideoEncoder, 2: use
     *                      MediaVideoBufferEncoder
     * @param previewFormat either UVCCamera.FRAME_FORMAT_YUYV(0) or UVCCamera.FRAME_FORMAT_MJPEG(1)
     */
    public static final UVCCameraHandler createHandler(
            final Activity parent, final CameraViewInterface cameraView,
            final int encoderType, final int previewWidth, final int previewHeight,
            final int previewFormat,
            final float previewBandwidthFactor,
            int recordWidth, int recordHeight, int recordFormat,
			int recordProfile, int recordUsage,
            float recordBandwidthFactor) {

        final CameraThread thread = new CameraThread(UVCCameraHandler.class, parent, cameraView,
                encoderType, previewWidth, previewHeight, previewFormat, previewBandwidthFactor,
                recordWidth, recordHeight, recordFormat, recordProfile,
				recordUsage, recordBandwidthFactor);
        thread.start();
        return (UVCCameraHandler) thread.getHandler();
    }

    protected UVCCameraHandler(final CameraThread thread) {
        super(thread);
    }

    @Override
    public void startPreview(final Object surface) {
        super.startPreview(surface);
    }

    @Override
    public void captureStill() {
        super.captureStill();
    }

    @Override
    public void captureStill(final String path) {
        super.captureStill(path);
    }
}
