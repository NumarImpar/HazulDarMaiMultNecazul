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
    private long[] _detectionsPtr;
    public volatile int[] _detectionIds = new int[3];
    public volatile int[] __ids = new int[3];
    public volatile int targetFound = -1;

    public int error = 0;

    private float decimation;
    private boolean _chDecimation = false;
    
    private Object lock1 = new Object();
    private Object lock2 = new Object();

    public AprilTagsPipeline(float initDecimation, int[] targets) {
        super();
	this.decimation = initDecimation;

        this.detectorPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, this.decimation, 2);
	System.arraycopy(targets, 0, this._detectionIds, 0, 3);
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

	if (input.empty()){return input;}
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);

	synchronized(this.lock1){
	    if(this._chDecimation) {
	        AprilTagDetectorJNI.setApriltagDetectorDecimation(this.detectorPtr, this.decimation);
		this._chDecimation = false;
	    }
	}

        long _detections = AprilTagDetectorJNI.runApriltagDetector(this.detectorPtr, input.nativeObj);
        this._detectionsPtr = ApriltagDetectionJNI.getDetectionPointers(_detections);
        
	synchronized(this.lock2) {
            for (int i = 0; i < this._detectionsPtr.length; i ++) {
	       if (_detectionsPtr[i] == 0){
	           this.__ids[i] = -1;
		   continue;
	       }
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
	return input;
    }


    public void kill(){
       AprilTagDetectorJNI.releaseApriltagDetector(this.detectorPtr);
    }
}

