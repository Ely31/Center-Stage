package org.firstinspires.ftc.teamcode.vision.workspace;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class JunctionPipeline extends OpenCvPipeline {
    // Constructor for eocv-sim telemetry
    public JunctionPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    // Normal constructor
    public JunctionPipeline(){}

    VisionUtil vUtil = new VisionUtil(Imgproc.COLOR_RGB2HSV);
    Telemetry telemetry = null;

    Mat binaryMat = new Mat();
    Mat rgbmat = new Mat();
    Mat displayMat = new Mat();

    private List<MatOfPoint> contours = new ArrayList<>();
    private RotatedRect[] rects;
    private double largestRectArea = 0;
    private RotatedRect largestRect = new RotatedRect();
    Point[] largestRectPoints = new Point[4];
    Point highestRectPoint = new Point(CameraConstants.width /2.0, CameraConstants.height -10);
    Point secondHighestRectPoint = new Point(CameraConstants.width /2.0, CameraConstants.height -30);
    Point topCenterPoint = new Point();

    // Constant stuff for the cleanMask method
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(4, 4));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

    // Tuneable values for the threshold
    public static Scalar lower = new Scalar(0, 141, 50);
    public static Scalar upper = new Scalar(30, 255, 200);

    // Configuration
    public static boolean openingEnabled = true;
    // Used in the getCorrectedJunctionXValue method
    public static double centerOffset = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Clear the old contours so they don't stack up
        contours.clear();
        // Reset the largest rect and rect area so it finds the new one
        largestRectArea = 0;
        largestRect = new RotatedRect(new Point(CameraConstants.width /2.0, 10), new Size(10,10), 0);
        // Reset the top two points for the same reason
        resetHighPoints();

        input.copyTo(rgbmat);
        input.copyTo(binaryMat);
        vUtil.prepareFrame(binaryMat);
        // Threshold
        Core.inRange(binaryMat, lower, upper, binaryMat);
        // Opening
        if (openingEnabled) cleanMask(binaryMat, binaryMat);
        displayMat.release();
        // Display what's underneath the thresholded mat
        Core.bitwise_and(rgbmat, rgbmat, displayMat, binaryMat);

        vUtil.getContours(binaryMat, contours);
        contours = vUtil.getFilteredContours(contours, 40, 2000);
        vUtil.drawContours(displayMat, contours, vUtil.red,2);

        // Fit rectangles to those contours we found and draw them all
        rects = new RotatedRect[contours.size()];
        for (int i=0; i < contours.size(); i++){
            rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
            vUtil.drawRotatedRect(rects[i], displayMat, vUtil.blue, 2);
        }

        // Find the largest rect
        for (RotatedRect rect : rects) {
            if (rect.size.height * rect.size.width > largestRectArea) {
                largestRectArea = rect.size.height * rect.size.width;
                largestRect = rect;
            }
        }
        // Draw the largest rect in green
        vUtil.drawRotatedRect(largestRect, displayMat, vUtil.green, 2);

        // Get the verticies of the largest rect
        largestRectPoints = new Point[4];
        largestRect.points(largestRectPoints);
        // Find the highest two
        for (int i = 0; i < largestRectPoints.length; i++) {
            // If higher than the highest, it is the highest
            if (largestRectPoints[i].y < highestRectPoint.y) {
                highestRectPoint = largestRectPoints[i];
            }
            // If higher than the second highest but lower than the highest, it is the second highest
            if (largestRectPoints[i].y < secondHighestRectPoint.y && (largestRectPoints[i].y > highestRectPoint.y)) {
                secondHighestRectPoint = largestRectPoints[i];
            }
        }
        // Mark the two highest points
        Imgproc.circle(displayMat, highestRectPoint, 2, vUtil.magenta, 2);
        Imgproc.circle(displayMat, secondHighestRectPoint, 2, vUtil.magenta, 2);
        // Get the midpoint of those top two points to find the center of the top of the junction
        topCenterPoint = calculateMidpoint(highestRectPoint, secondHighestRectPoint);

        // Mark the point with a circle
        Imgproc.circle(displayMat, topCenterPoint, 4, vUtil.white, 2);

        printTelemetry();
        return displayMat;
    }

    // Copied from an old message by NPE on the ftc discord
    void cleanMask(Mat input, Mat output)
    {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void resetHighPoints(){
        highestRectPoint = new Point(CameraConstants.width /2.0, CameraConstants.height -10);
        secondHighestRectPoint = new Point(CameraConstants.width /2.0, CameraConstants.height -20);
    }

    Point calculateMidpoint(Point p1, Point p2){
        return (new Point((p1.x + p2.x) /2.0, (p1.y + p2.y) /2.0));
    }

    double getJunctionXValue(){
        return topCenterPoint.x;
    }
    public double getCorrectedJunctionXValue(){
        return (getJunctionXValue() - (CameraConstants.width /2.0)) + centerOffset;
    }

    void printTelemetry(){
        telemetry.addData("highest point y", highestRectPoint.y);
        telemetry.addData("highest point x", highestRectPoint.x);
        telemetry.addData("second highest point y", secondHighestRectPoint.y);
        telemetry.addData("second highest point x", secondHighestRectPoint.x);
        telemetry.addData("corrected x val", getCorrectedJunctionXValue());
        telemetry.update();
    }
}
