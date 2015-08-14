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
			Log.d("SCREEN","IMMERSIVE MODE ACTIVE");

			getWindow().getDecorView().
			setSystemUiVisibility(
			View.SYSTEM_UI_FLAG_LAYOUT_STABLE
			| View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
			| View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
			| View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
			| View.SYSTEM_UI_FLAG_FULLSCREEN
			| View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);

	}

	if (!QtApplication.invokeDelegate(hasFocus).invoked)
		super.onWindowFocusChanged(hasFocus);
     }
}
