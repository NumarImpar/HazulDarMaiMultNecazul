package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.ApriltagDetectionJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;

/*
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠛⠻⢿⣿⣿⣿⣿⣿⣿⠟⠀⠈⣻⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⣄⠀⠀⠈⠛⢿⣿⡿⠁⠀⣠⡾⢋⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣀⠙⠻⣦⣄⡀⠀⠈⠛⢶⣴⡋⣠⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣤⣀⣽⠿⢶⣄⡀⠀⠈⠛⢿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠋⠀⢀⣿⣿⠷⣤⣀⠀⠈⠙⢿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠀⠀⣴⡿⢉⣿⣄⡈⠙⠷⣦⣴⡿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠁⠀⣠⣾⠋⣴⣿⣿⣿⣿⣶⣤⡀⣿⣼⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⠋⠀⢀⣴⠟⣡⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⠟⠁⠀⣠⡿⢋⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⡟⠁⠀⢀⣾⠟⣰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⡿⠋⠀⠀⣴⡿⢡⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⡁⠀⣠⣾⢋⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⡏⠛⣿⠟⣵⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣶⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 */

public class AprilTagsPipeline extends OpenCvPipeline {
    private long detectorPtr;
    private Mat toGray = new Mat();
    private long[] _detectionsPtr;
    public volatile int[] _detectionIds = new int[3];
    public volatile int[] __ids = new int[3];
    public volatile int targetFound = -1;

    public int error = 0;

    private float decimation;
    private boolean _chDecimation = false;
    
    private Object lock1 = new Object();
    private Object lock2 = new Object();

    public Mat __ = new Mat();

    public AprilTagsPipeline(float initDecimation, int[] targets) {
        this.decimation = initDecimation;

        this.detectorPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, this.decimation, 2);
	System.arraycopy(targets, 0, this._detectionIds, 0, 3);
    }

    @Override
    public void init(Mat input){
        if (this.detectorPtr == 0){
	    this.error = 1;
	    this.kill();
	}
    }

    public void setTargets(int[] targets){ // praying this works
	synchronized(this.lock2){
	    System.arraycopy(targets, 0, this._detectionIds, 0, 3);
	}
    }

    public float getDecimation(){
        return this.decimation;
    }

    public void setDecimation(float _decimation){
        this.decimation = _decimation;
	this._chDecimation = true;
    }

    @Override
    public Mat processFrame(Mat input){
	if (this.error != 0){
	    return input;
	}

        Imgproc.cvtColor(input, this.toGray, Imgproc.COLOR_RGB2GRAY);

	synchronized(this.lock1){
	    if(this._chDecimation) {
	        AprilTagDetectorJNI.setApriltagDetectorDecimation(this.detectorPtr, this.decimation);
		this._chDecimation = false;
	    }
	}

        long _detections = AprilTagDetectorJNI.runApriltagDetector(this.detectorPtr, this.toGray.nativeObj);
        this._detectionsPtr = ApriltagDetectionJNI.getDetectionPointers(_detections);
        
	synchronized(this.lock2) {
            for (int i = 0; i < this._detectionsPtr.length; i ++) {
	       int id = ApriltagDetectionJNI.getId(this._detectionsPtr[i]);
               if (i < 2){
	           this.__ids[i] = id;
	       }
	       if (id == this._detectionIds[0]){
	          if (this.targetFound != -1){
	              this.targetFound = -2;
	          } else {
	              this.targetFound = 0;
	          }
	       } else if (id == this._detectionIds[1]) {
	          if (this.targetFound != -1){
	              this.targetFound = -2;
	          } else {
	              this.targetFound = 1;
	          }
	       } else if (id == this._detectionIds[2]) {
	          if (this.targetFound != -1){
	              this.targetFound = -2;
	          } else {
	              this.targetFound = 2;
	          }
	       } 
	    }
        }

	ApriltagDetectionJNI.freeDetectionList(_detections);
	return __;
    }


    public void kill(){
       this.__.release();
       this.toGray.release();
       AprilTagDetectorJNI.releaseApriltagDetector(this.detectorPtr);
    }
}

