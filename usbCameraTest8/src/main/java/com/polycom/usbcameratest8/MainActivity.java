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

package com.polycom.usbcameratest8;

import android.animation.Animator;
import android.graphics.SurfaceTexture;
import android.hardware.usb.UsbDevice;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.util.Log;
import android.view.Surface;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.View.OnLongClickListener;
import android.view.Window;
import android.view.WindowManager;
import android.widget.CompoundButton;
import android.widget.ImageButton;
import android.widget.SeekBar;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.polycom.usbcameracommon.UVCCameraHandler;
import com.polycom.uvccamera.common.BaseActivity;
import com.polycom.uvccamera.usb.CameraDialog;
import com.polycom.uvccamera.usb.USBMonitor;
import com.polycom.uvccamera.usb.USBMonitor.OnDeviceConnectListener;
import com.polycom.uvccamera.usb.USBMonitor.UsbControlBlock;
import com.polycom.uvccamera.usb.UVCCamera;
import com.polycom.widget.CameraViewInterface;
import com.serenegiant.utils.ViewAnimationHelper;

public final class MainActivity extends BaseActivity implements CameraDialog.CameraDialogParent {
    protected static final int SETTINGS_HIDE_DELAY_MS = 2500;
    private static final boolean DEBUG = true;    // TODO set false on release
    private static final String TAG = "MainActivity";
    /**
     * set true if you want to record movie using MediaSurfaceEncoder
     * (writing frame data into Surface camera from MediaCodec
     * by almost same way as USBCameratest2)
     * set false if you want to record movie using MediaVideoEncoder
     */
    private static final boolean USE_SURFACE_ENCODER = false;
    /**
     * preview resolution(width)
     * if your camera does not support specific resolution and mode,
     * {@link UVCCamera#setPreviewSize(int, int, int)} throw exception
     */
    private static final int PREVIEW_WIDTH = 1280;
    /**
     * preview resolution(height)
     * if your camera does not support specific resolution and mode,
     * {@link UVCCamera#setPreviewSize(int, int, int)} throw exception
     */
    private static final int PREVIEW_HEIGHT = 720;
    /**
     * preview mode
     * if your camera does not support specific resolution and mode,
     * {@link UVCCamera#setPreviewSize(int, int, int)} throw exception
     * 0:YUYV, 1:MJPEG, 2:NV12
     */
    private static final int PREVIEW_MODE = 1;
    /**
     * for accessing USB
     */
    private USBMonitor mUSBMonitor;
    /**
     * Handler to execute camera related methods sequentially on private thread
     */
    private UVCCameraHandler mCameraHandler;
    /**
     * capture still image when you long click on preview image(not on buttons)
     */
    private final OnLongClickListener mOnLongClickListener = new OnLongClickListener() {
        @Override
        public boolean onLongClick(final View view) {
            switch (view.getId()) {
                case R.id.camera_view:
                    if (mCameraHandler.isOpened()) {
                        if (checkPermissionWriteExternalStorage()) {
                            mCameraHandler.captureStill();
                        }
                        return true;
                    }
            }
            return false;
        }
    };
    /**
     * for camera preview display
     */
    private CameraViewInterface mUVCCameraView;
    /**
     * for open&start / stop&close camera preview
     */
    private ToggleButton mCameraButton;
    /**
     * button for start/stop recording
     */
    private ImageButton mCaptureButton;
    private View mBrightnessButton, mContrastButton;
    private View mZoomButton, mPanButtion, mTiltButton;
    private View mResetButton;
    private View mToolsLayout, mValueLayout;
    private final Runnable mUpdateItemsOnUITask = new Runnable() {
        @Override
        public void run() {
            if (isFinishing()) return;
            final int visible_active = isActive() ? View.VISIBLE : View.INVISIBLE;
            mToolsLayout.setVisibility(visible_active);

            mBrightnessButton.setVisibility(
                    checkSupportFlag(UVCCamera.PU_BRIGHTNESS_NAME)
                            ? visible_active : View.INVISIBLE);
            mContrastButton.setVisibility(
                    checkSupportFlag(UVCCamera.PU_CONTRAST_NAME)
                            ? visible_active : View.INVISIBLE);
            mZoomButton.setVisibility(
                    checkSupportFlag(UVCCamera.CTRL_ZOOM_ABS_NAME)
                            ? visible_active : View.INVISIBLE);
            mPanButtion.setVisibility(
                    checkSupportFlag(UVCCamera.CTRL_PAN_ABS_NAME)
                            ? visible_active : View.INVISIBLE);
            mTiltButton.setVisibility(
                    checkSupportFlag(UVCCamera.CTRL_TILT_ABS_NAME)
                            ? visible_active : View.INVISIBLE);

        }
    };
    private final CompoundButton.OnCheckedChangeListener mOnCheckedChangeListener
            = new CompoundButton.OnCheckedChangeListener() {
        @Override
        public void onCheckedChanged(final CompoundButton compoundButton, final boolean isChecked) {
            switch (compoundButton.getId()) {
                case R.id.camera_button:
                    if (isChecked && !mCameraHandler.isOpened()) {
                        CameraDialog.showDialog(MainActivity.this);
                    } else {
                        mCameraHandler.close();
                        setCameraButton(false);
                    }
                    break;
            }
        }
    };
    private final OnDeviceConnectListener mOnDeviceConnectListener = new OnDeviceConnectListener() {
        @Override
        public void onAttach(final UsbDevice device) {
            Toast.makeText(MainActivity.this, "USB_DEVICE_ATTACHED", Toast.LENGTH_SHORT).show();
        }

        @Override
        public void onConnect(final UsbDevice device, final UsbControlBlock ctrlBlock,
                              final boolean createNew) {
            if (DEBUG) Log.v(TAG, "onConnect:");
            mCameraHandler.open(ctrlBlock);
            startPreview();
            updateItems();
        }

        @Override
        public void onDisconnect(final UsbDevice device, final UsbControlBlock ctrlBlock) {
            if (DEBUG) Log.v(TAG, "onDisconnect:");
            if (mCameraHandler != null) {
                queueEvent(new Runnable() {
                    @Override
                    public void run() {
                        mCameraHandler.close();
                    }
                }, 0);
                setCameraButton(false);
                updateItems();
            }
        }

        @Override
        public void onDettach(final UsbDevice device) {
            Toast.makeText(MainActivity.this, "USB_DEVICE_DETACHED", Toast.LENGTH_SHORT).show();
        }

        @Override
        public void onCancel(final UsbDevice device) {
            setCameraButton(false);
        }
    };
    private SeekBar mSettingSeekbar;
    private int mSettingMode = -1;
    /**
     * 設定値変更用のシークバーのコールバックリスナー
     */
    private final SeekBar.OnSeekBarChangeListener mOnSeekBarChangeListener =
            new SeekBar.OnSeekBarChangeListener() {
                @Override
                public void onProgressChanged(final SeekBar seekBar, final int progress,
                                              final boolean fromUser) {
                    // 設定が変更された時はシークバーの非表示までの時間を延長する
                    if (fromUser) {
                        runOnUiThread(mSettingHideTask, SETTINGS_HIDE_DELAY_MS);
                        if (isActive()) {
                            switch (mSettingMode) {
                                case UVCCamera.PU_BRIGHTNESS:
                                    if (checkSupportFlag(UVCCamera.PU_BRIGHTNESS_NAME))
                                        setValue(mSettingMode, seekBar.getProgress());
                                    break;
                                case UVCCamera.PU_CONTRAST:
                                    if (checkSupportFlag(UVCCamera.PU_CONTRAST_NAME))
                                        setValue(mSettingMode, seekBar.getProgress());
                                    break;
                                case UVCCamera.CTRL_ZOOM_ABS:
                                    if (checkSupportFlag(UVCCamera.CTRL_ZOOM_ABS_NAME))
                                        setValue(mSettingMode, seekBar.getProgress());
                                    break;
                                case UVCCamera.CTRL_PAN_ABS:
                                    if (checkSupportFlag(UVCCamera.CTRL_PAN_ABS_NAME))
                                        setValue(mSettingMode, seekBar.getProgress());
                                    break;
                                case UVCCamera.CTRL_TILT_ABS:
                                    if (checkSupportFlag(UVCCamera.CTRL_TILT_ABS_NAME))
                                        setValue(mSettingMode, seekBar.getProgress());
                                    break;
                            }
                        }
                    }
                }

                @Override
                public void onStartTrackingTouch(final SeekBar seekBar) {
                }

                @Override
                public void onStopTrackingTouch(final SeekBar seekBar) {
                    // シークバーにタッチして値を変更した時はonProgressChangedへ
                    // 行かないみたいなのでここでも非表示までの時間を延長する
                    runOnUiThread(mSettingHideTask, SETTINGS_HIDE_DELAY_MS);
                }
            };
    private final ViewAnimationHelper.ViewAnimationListener
            mViewAnimationListener = new ViewAnimationHelper.ViewAnimationListener() {
        @Override
        public void onAnimationStart(@NonNull final Animator animator, @NonNull final View target,
                                     final int animationType) {
//			if (DEBUG) Log.v(TAG, "onAnimationStart:");
        }

        @Override
        public void onAnimationEnd(@NonNull final Animator animator, @NonNull final View target,
                                   final int animationType) {
            final int id = target.getId();
            switch (animationType) {
                case ViewAnimationHelper.ANIMATION_FADE_IN:
                case ViewAnimationHelper.ANIMATION_FADE_OUT: {
                    final boolean fadeIn = animationType == ViewAnimationHelper.ANIMATION_FADE_IN;
                    if (id == R.id.value_layout) {
                        if (fadeIn) {
                            runOnUiThread(mSettingHideTask, SETTINGS_HIDE_DELAY_MS);
                        } else {
                            mValueLayout.setVisibility(View.GONE);
                            mSettingMode = -1;
                        }
                    } else if (!fadeIn) {
//					target.setVisibility(View.GONE);
                    }
                    break;
                }
            }
        }

        @Override
        public void onAnimationCancel(@NonNull final Animator animator, @NonNull final View target,
                                      final int animationType) {
//			if (DEBUG) Log.v(TAG, "onAnimationStart:");
        }
    };
    protected final Runnable mSettingHideTask = new Runnable() {
        @Override
        public void run() {
            hideSetting(true);
        }
    };
    /**
     * event handler when click camera / capture button
     */
    private final OnClickListener mOnClickListener = new OnClickListener() {
        int count = 0;

        @Override
        public void onClick(final View view) {
            switch (view.getId()) {
                case R.id.capture_button:
                    if (mCameraHandler.isOpened()) {
                        if (checkPermissionWriteExternalStorage() && checkPermissionAudio()) {
                            if (!mCameraHandler.isRecording()) {
                                mCaptureButton.setColorFilter(0xffff0000);    // turn red
                                setRecordVideoParams("encoder_select_layer=0x0000," +
                                        "encoder_h_264_profile_toolset=0x4D00," +
                                        "encoder_select_layer=0x0000," +
                                        "encoder_average_bitrate=4096000");
                                mCameraHandler.startRecord();
                            } else {
                                if (count++ > 5) {
                                    mCaptureButton.setColorFilter(0);    // return to default color
                                    mCameraHandler.stopRecord();
                                    count = 0;
                                } else {
                                    setValue(UVCCamera.EU_SYNC_REF_FRAME, 1);
                                }
                            }
                        }
                    }
                    break;
                case R.id.brightness_button:
                    showSettings(UVCCamera.PU_BRIGHTNESS);
                    break;
                case R.id.contrast_button:
                    showSettings(UVCCamera.PU_CONTRAST);
                    break;
                case R.id.zoom_button:
                    showSettings(UVCCamera.CTRL_ZOOM_ABS);
                    break;
                case R.id.pan_button:
                    showSettings(UVCCamera.CTRL_PAN_ABS);
                    break;
                case R.id.tilt_button:
                    showSettings(UVCCamera.CTRL_TILT_ABS);
                    break;
                case R.id.reset_button:
                    resetSettings();
                    break;
            }
        }
    };

    @Override
    protected void onCreate(final Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (DEBUG) Log.v(TAG, "onCreate:");
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_main);
        mCameraButton = (ToggleButton) findViewById(R.id.camera_button);
        mCameraButton.setOnCheckedChangeListener(mOnCheckedChangeListener);
        mCaptureButton = (ImageButton) findViewById(R.id.capture_button);
        mCaptureButton.setOnClickListener(mOnClickListener);
        mCaptureButton.setVisibility(View.INVISIBLE);
        final View view = findViewById(R.id.camera_view);
        view.setOnLongClickListener(mOnLongClickListener);
        mUVCCameraView = (CameraViewInterface) view;
        mUVCCameraView.setAspectRatio(PREVIEW_WIDTH / (float) PREVIEW_HEIGHT);

        mBrightnessButton = findViewById(R.id.brightness_button);
        mBrightnessButton.setOnClickListener(mOnClickListener);
        mContrastButton = findViewById(R.id.contrast_button);
        mContrastButton.setOnClickListener(mOnClickListener);
        mZoomButton = findViewById(R.id.zoom_button);
        mZoomButton.setOnClickListener(mOnClickListener);
        mPanButtion = findViewById(R.id.pan_button);
        mPanButtion.setOnClickListener(mOnClickListener);
        mTiltButton = findViewById(R.id.tilt_button);
        mTiltButton.setOnClickListener(mOnClickListener);
        mResetButton = findViewById(R.id.reset_button);
        mResetButton.setOnClickListener(mOnClickListener);
        mSettingSeekbar = (SeekBar) findViewById(R.id.setting_seekbar);
        mSettingSeekbar.setOnSeekBarChangeListener(mOnSeekBarChangeListener);

        mToolsLayout = findViewById(R.id.tools_layout);
        mToolsLayout.setVisibility(View.INVISIBLE);
        mValueLayout = findViewById(R.id.value_layout);
        mValueLayout.setVisibility(View.INVISIBLE);

        mUSBMonitor = new USBMonitor(this, mOnDeviceConnectListener);
        mCameraHandler = UVCCameraHandler.createHandler(this, mUVCCameraView,
                USE_SURFACE_ENCODER ? 0 : 1, PREVIEW_WIDTH, PREVIEW_HEIGHT, PREVIEW_MODE);
    }

    @Override
    protected void onStart() {
        super.onStart();
        if (DEBUG) Log.v(TAG, "onStart:");
        mUSBMonitor.register();
        if (mUVCCameraView != null) {
            mUVCCameraView.onResume();
        }
    }

    @Override
    protected void onStop() {
        if (DEBUG) Log.v(TAG, "onStop:");
        mCameraHandler.close();
        if (mUVCCameraView != null) {
            mUVCCameraView.onPause();
        }
        setCameraButton(false);
        super.onStop();
    }

    @Override
    public void onDestroy() {
        if (DEBUG) Log.v(TAG, "onDestroy:");
        if (mCameraHandler != null) {
            mCameraHandler.release();
            mCameraHandler = null;
        }
        if (mUSBMonitor != null) {
            mUSBMonitor.destroy();
            mUSBMonitor = null;
        }
        mUVCCameraView = null;
        mCameraButton = null;
        mCaptureButton = null;
        super.onDestroy();
    }

    private void setCameraButton(final boolean isOn) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (mCameraButton != null) {
                    try {
                        mCameraButton.setOnCheckedChangeListener(null);
                        mCameraButton.setChecked(isOn);
                    } finally {
                        mCameraButton.setOnCheckedChangeListener(mOnCheckedChangeListener);
                    }
                }
                if (!isOn && (mCaptureButton != null)) {
                    mCaptureButton.setVisibility(View.INVISIBLE);
                }
            }
        }, 0);
        updateItems();
    }

    private void startPreview() {
        final SurfaceTexture st = mUVCCameraView.getSurfaceTexture();
        mCameraHandler.startPreview(new Surface(st));
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mCaptureButton.setVisibility(View.VISIBLE);
            }
        });
        updateItems();
    }

    /**
     * to access from CameraDialog
     */
    @Override
    public USBMonitor getUSBMonitor() {
        return mUSBMonitor;
    }

    @Override
    public void onDialogResult(boolean canceled) {
        if (DEBUG) Log.v(TAG, "onDialogResult:canceled=" + canceled);
        if (canceled) {
            setCameraButton(false);
        }
    }

    //================================================================================
    private boolean isActive() {
        return mCameraHandler != null && mCameraHandler.isOpened();
    }

    private boolean checkSupportFlag(final String ctrlName) {
        return mCameraHandler != null && mCameraHandler.checkSupportFlag(ctrlName);
    }

    private int setRecordVideoParams(final String ctrlSets) {
        return mCameraHandler != null ? mCameraHandler.setVideoParams(ctrlSets) : -1;
    }

    private int getValue(final int flag) {
        return mCameraHandler != null ? mCameraHandler.getValue(flag) : 0;
    }

    private int setValue(final int flag, final int value) {
        return mCameraHandler != null ? mCameraHandler.setValue(flag, value) : 0;
    }

    private int resetValue(final int flag) {
        return mCameraHandler != null ? mCameraHandler.resetValue(flag) : 0;
    }

    private void updateItems() {
        runOnUiThread(mUpdateItemsOnUITask, 400);
    }

    /**
     * 設定画面を表示
     */
    private final void showSettings(final int mode) {
        if (DEBUG) Log.v(TAG, String.format("showSettings:%08x", mode));
        hideSetting(false);
        if (isActive()) {
            switch (mode) {
                case UVCCamera.PU_BRIGHTNESS:
                case UVCCamera.PU_CONTRAST:
                case UVCCamera.CTRL_ZOOM_ABS:
                case UVCCamera.CTRL_PAN_ABS:
                case UVCCamera.CTRL_TILT_ABS:
                    mSettingMode = mode;
                    mSettingSeekbar.setProgress(getValue(mode));
                    ViewAnimationHelper.fadeIn(mValueLayout, -1, 0, mViewAnimationListener);
                    break;
            }
        }
    }

    private void resetSettings() {
        if (isActive()) {
            switch (mSettingMode) {
                case UVCCamera.PU_BRIGHTNESS:
                case UVCCamera.PU_CONTRAST:
                case UVCCamera.CTRL_ZOOM_ABS:
                case UVCCamera.CTRL_PAN_ABS:
                case UVCCamera.CTRL_TILT_ABS:
                    mSettingSeekbar.setProgress(resetValue(mSettingMode));
                    break;
            }
        }
        mSettingMode = -1;
        ViewAnimationHelper.fadeOut(mValueLayout, -1, 0, mViewAnimationListener);
    }

    /**
     * 設定画面を非表示にする
     *
     * @param fadeOut trueならばフェードアウトさせる, falseなら即座に非表示にする
     */
    protected final void hideSetting(final boolean fadeOut) {
        removeFromUiThread(mSettingHideTask);
        if (fadeOut) {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    ViewAnimationHelper.fadeOut(mValueLayout, -1, 0, mViewAnimationListener);
                }
            }, 0);
        } else {
            try {
                mValueLayout.setVisibility(View.GONE);
            } catch (final Exception e) {
                // ignore
            }
            mSettingMode = -1;
        }
    }

}
