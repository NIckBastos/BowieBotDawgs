package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Camera;
import android.util.AttributeSet;

import org.opencv.android.JavaCameraView;

import java.util.List;

/**
 * Created by Molly on 9/5/2016.
 * This class provides a hook to adjust the camera settings used by OpenCV
 */
public class MyJavaCameraView extends JavaCameraView {

    /*
     * Flag used to determine whether the full camera resolution is passed to the image processor
     * The full resolution givesa more detail and can identify the beacons from across the field,
     * however the full image is not displayed on the screen.
     * Using the default screen resolution limits how far away the beacons can be detected, but
     * the entire image is visible.
     */
    private boolean useFullResolution;
    static int fullResolutionWidth;
    static int fullResolutionHeight;

    public MyJavaCameraView(Context context, AttributeSet attrs){
        super(context,attrs);
        fullResolutionHeight = 1;
        fullResolutionWidth = 1;
    }

    public MyJavaCameraView(Context context, int cameraId) {
        super(context, cameraId);
    }

    public void setFullResolution(boolean flag){
        useFullResolution = flag;
    }

    @Override
    protected boolean initializeCamera(int width, int height){
        boolean retval = false;
        if (useFullResolution) {
            retval = super.initializeCamera(1280, 960);
        } else {
            retval = super.initializeCamera(width, height);
        }

        // figure out the maximum resolution available
        Camera.Parameters params = mCamera.getParameters();
        List<Camera.Size> sizes = params.getSupportedPreviewSizes();
        for (Camera.Size size : sizes) {
            if (size.width > fullResolutionWidth) {
                fullResolutionWidth = size.width;
                fullResolutionHeight = size.height;
            }
        }
        setMinExposure();
        return retval;
    }

    public static int getFullResolutionWidth() { return fullResolutionWidth;}

    public static int getFullResolutionHeight() {return fullResolutionHeight;}

    public void setMinExposure(){
        if(mCamera == null) return;
        Camera.Parameters params = mCamera.getParameters();
        if(params.isAutoExposureLockSupported()){
            params.setAutoExposureLock(true);
            params.setExposureCompensation(params.getMinExposureCompensation());
        }

        mCamera.setParameters(params);
    }

    protected void disconnectCamera() {
        super.disconnectCamera();
    }

    protected boolean connectCamera(int width, int height) {
        if(useFullResolution){
            return super.connectCamera(fullResolutionWidth, fullResolutionHeight);
        }else {
            // find out the size of the layout and get the largest frame available which
            // fits that frame
            return super.connectCamera(width, height);
        }
    }
}
