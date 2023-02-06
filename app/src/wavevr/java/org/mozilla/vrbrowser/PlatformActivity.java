/* -*- Mode: Java; c-basic-offset: 4; tab-width: 4; indent-tabs-mode: nil; -*-
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

package com.igalia.wolvic;

import com.fulldive.encoder.AESUtils;
import com.htc.vr.sdk.VRActivity;

import android.Manifest;
import android.content.res.AssetManager;
import android.os.Bundle;
import android.view.KeyEvent;

import com.igalia.wolvic.utils.SystemUtils;
import com.fulldive.encoder.TagReader;
import android.util.Log;

public class PlatformActivity extends VRActivity {
    static String LOGTAG = SystemUtils.createLogtag(PlatformActivity.class);
    private TagReader mTagReader;
    private static final byte[] DEBUG_API_KEY = new byte[] {(byte)0xBC, (byte)0xA9, (byte)0x1B, (byte)0xD1, (byte)0xA8, (byte)0xAB, (byte)0x28, (byte)0x8D, (byte)0xDF, (byte)0x03, (byte)0x4D, (byte)0x65, (byte)0xBE, (byte)0x09, (byte)0xD6, (byte)0xD2};

    public static boolean filterPermission(final String aPermission) {
        if (aPermission.equals(Manifest.permission.CAMERA)) {
            return true;
        }
        return false;
    }

    public static boolean isNotSpecialKey(KeyEvent event) {
        return true;
    }

    public static boolean isPositionTrackingSupported() {
        // Dummy implementation.
        return true;
    }

    public PlatformActivity() {}

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mTagReader = TagReader.Companion.getInstance();

        try {
            mTagReader.read(AESUtils.Companion.toHex(DEBUG_API_KEY));
        }catch (Exception ex){
            ex.printStackTrace();
            Log.e("fftf", ex.getMessage());
        }

        queueRunnable(new Runnable() {
            @Override
            public void run() {
                initializeJava(getAssets());
            }
        });
    }

    @Override
    public void onBackPressed() {
        // Discard back button presses that would otherwise exit the app,
        // as the UI standard on this platform is to require the use of
        // the system menu to exit applications.
    }

    protected native void queueRunnable(Runnable aRunnable);
    protected native void initializeJava(AssetManager aAssets);
}
