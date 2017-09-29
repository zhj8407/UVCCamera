package com.polycom.uvccamera.utils;

/**
 * Created by jiezhang on 17-9-19.
 */

import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;

public class HandlerThreadHandler extends Handler {
    private static final String TAG = "HandlerThreadHandler";

    private HandlerThreadHandler(final Looper looper) {
        super(looper);
    }

    private HandlerThreadHandler(final Looper looper, final Callback callback) {
        super(looper, callback);
    }

    public static final HandlerThreadHandler createHandler() {
        return createHandler(TAG);
    }

    public static final HandlerThreadHandler createHandler(final String name) {
        final HandlerThread thread = new HandlerThread(name);
        thread.start();
        return new HandlerThreadHandler(thread.getLooper());
    }

    public static final HandlerThreadHandler createHandler(final Callback callback) {
        return createHandler(TAG, callback);
    }

    public static final HandlerThreadHandler createHandler(final String name,
            final Callback callback) {
        final HandlerThread thread = new HandlerThread(name);
        thread.start();
        return new HandlerThreadHandler(thread.getLooper(), callback);
    }

}
