
package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.ApriltagDetectionJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;

public class NewPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input){
	    Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);
	    return input;}
}
