package org.qtproject.qt5.android.bindings;

import org.qtproject.qt5.android.bindings.QtActivity;

import android.os.Build;

import android.view.View;

import android.util.Log;

public class StartActivity extends QtActivity
{

     public StartActivity(){}

     @Override

     public void onWindowFocusChanged(boolean hasFocus)
     {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.KITKAT) {

                        if (hasFocus) {

                            Log.d("SCREEN","IMMERSIVE MODE ACTIVE");

                                    getWindow().getDecorView().
                            setSystemUiVisibility(
                            View.
                            SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
                        }
                }

         if (!QtApplication.invokeDelegate(hasFocus).invoked)
             super.onWindowFocusChanged(hasFocus);
     }
}


// http://grokbase.com/t/gg/android-qt/13c9try9xt/how-to-activate-immersive-full-screen-mode-in-android-4-4
