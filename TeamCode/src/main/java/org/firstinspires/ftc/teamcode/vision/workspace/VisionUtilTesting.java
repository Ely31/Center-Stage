package org.firstinspires.ftc.teamcode.vision.workspace;

import org.firstinspires.ftc.teamcode.vision.workspace.VisionUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class VisionUtilTesting extends OpenCvPipeline {
    VisionUtil visionUtil = new VisionUtil(Imgproc.COLOR_RGB2YCrCb);

    private Mat processedMat = new Mat();
    private Mat displayMat = new Mat();

    public Scalar lower = new Scalar(0, 0, 65);
    public Scalar upper = new Scalar(255, 185, 102);

    private List<MatOfPoint> contours = new ArrayList<>();
    private RotatedRect[] fittedRects;

    @Override
    public Mat processFrame(Mat input) {
        visionUtil.prepareFrame(input);
        input.copyTo(displayMat);
        Imgproc.cvtColor(displayMat, displayMat, Imgproc.COLOR_YCrCb2RGB);

        Core.inRange(input, lower, upper, processedMat);

        contours  = visionUtil.getContours(processedMat, contours);
        contours  = visionUtil.getFilteredContours(contours, 20, 500);

        visionUtil.drawContours(displayMat, contours, visionUtil.red);

        contours.clear();

        return displayMat;
    }
}
