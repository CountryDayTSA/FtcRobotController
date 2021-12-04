package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.os.Build;

import androidx.annotation.RequiresApi;

import com.intel.realsense.librealsense.DepthFrame;
import com.intel.realsense.librealsense.DeviceListener;
import com.intel.realsense.librealsense.Extension;
import com.intel.realsense.librealsense.Frame;
import com.intel.realsense.librealsense.FrameSet;
import com.intel.realsense.librealsense.Pipeline;
import com.intel.realsense.librealsense.RsContext;
import com.intel.realsense.librealsense.StreamType;

public class RealSense implements Runnable{
    private float currentDepth;
    private final Context appContext;
    private final Thread mStreamingThread = new Thread(new Runnable() {
        @RequiresApi(api = Build.VERSION_CODES.N)
        @Override
        public void run() {
            try {
                stream();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    });
    //constructor, need the app Context;
    public RealSense(Context appContext){
        this.appContext = appContext;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)

    public void run(){
        RsContext.init(appContext);
        RsContext mRsContext = new RsContext();
        mRsContext.setDevicesChangedCallback(new DeviceListener() {
            @Override
            public void onDeviceAttach() {
                mStreamingThread.start();
            }

            @Override
            public void onDeviceDetach() {
                mStreamingThread.interrupt();
            }
        });
    }
    public void stop(){
        mStreamingThread.interrupt();
    }

    public float getDepth(){
        return this.currentDepth;
    }


    private void stream() throws Exception {
        Pipeline pipe = new Pipeline();
        pipe.start();
        while (!mStreamingThread.isInterrupted())
        {
            try (FrameSet frames = pipe.waitForFrames()) {
                try (Frame f = frames.first(StreamType.DEPTH))
                {
                    DepthFrame depth = f.as(Extension.DEPTH_FRAME);
                    final float deptValue = depth.getDistance(depth.getWidth()/2, depth.getHeight()/2);
                    this.currentDepth = deptValue;
                }
            }
        }
        pipe.stop();
    }
}
