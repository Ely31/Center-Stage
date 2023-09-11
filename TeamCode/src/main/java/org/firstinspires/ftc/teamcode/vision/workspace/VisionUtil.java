package org.firstinspires.ftc.teamcode.vision.workspace;

import static org.firstinspires.ftc.teamcode.vision.workspace.CameraConstants.height;
import static org.firstinspires.ftc.teamcode.vision.workspace.CameraConstants.width;
import static org.firstinspires.ftc.teamcode.vision.workspace.CameraConstants.pixelsPerHorizontalDegree;
import static org.firstinspires.ftc.teamcode.vision.workspace.CameraConstants.pixelsPerVerticalDegree;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;


// Class to reduce boilerplate in vision pipelines by providing methods for common operations
public class VisionUtil {
    private final Mat hierarchy = new Mat();
    private int colorSpace; // YCrCb is 37 and HSV is 41
    // Constructor
    public VisionUtil(int colorSpace){
        this.colorSpace = colorSpace;
    }

    // Color constants
    public Scalar
            red = new Scalar(       255,0,  0),
            orange = new Scalar(    255,130,0),
            yellow = new Scalar(    255,255,0),
            green = new Scalar(     0,  255,0),
            cyan = new Scalar(      0,  255,255),
            blue = new Scalar(      0,  0,  255),
            magenta = new Scalar(   255,0,  255),
            black = new Scalar(     0,  0,  0),
            white = new Scalar(     255,255,255);

    // Blur a little and convert to YCrCb
    public void prepareFrame(Mat inputMat){
        Imgproc.blur(inputMat, inputMat, new Size(2,2));
        Imgproc.cvtColor(inputMat, inputMat, colorSpace);
    }

    // Contour stuff
    public List<MatOfPoint> getContours(Mat inputmat, List<MatOfPoint> contours){
        Imgproc.findContours(inputmat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }
    // Filters countours that are too small or too big
    public List<MatOfPoint> getFilteredContours(List<MatOfPoint> contours, int minLength, int maxLength){
        for (int i = 0; i < contours.size(); i++) {
            // If the length is less than the min or greater than the max, delete that contour
            if (contours.get(i).rows() < minLength || contours.get(i).rows() > maxLength){
                contours.remove(i);
                i--; // This line is hacky but I think it should work
            }
        }
        return contours;
    }

    // Simplify the eocv method a little bit
    public void drawContours(Mat outputmat, List<MatOfPoint> contours, Scalar color, int thickness){
        Imgproc.drawContours(outputmat, contours, -1, color, thickness);
    }
    public void drawContours(Mat outputmat, List<MatOfPoint> contours, Scalar color){
        drawContours(outputmat, contours, color, 1);
    }

    // Scoring methods
    // In the scoring system, 0 is a perfect score and all scores are positive
    // The higher the score, the worse it is
    public double getAreaScore(RotatedRect rect, MatOfPoint contour, double idealScore){
        return Math.abs(idealScore - ((rect.size.width*rect.size.height) / Imgproc.contourArea(contour)));
    }
    // Eccentricity is the length longest side divided by the length of the shortest side
    public double getEccentricityScore(RotatedRect rect, double idealScore){
        return Math.abs(idealScore - (Math.max(rect.size.height / rect.size.width, rect.size.width / rect.size.height)));
    }

    // Why is drawing rotated rects so hard
    public void drawRotatedRect(RotatedRect rect, Mat outputMat, Scalar color, int thickness){
        Point[] rectPoints = new Point[4];
        rect.points(rectPoints);
        for (int i = 0; i < 4; i++) {
            Imgproc.line(outputMat, rectPoints[i], rectPoints[(i+1) % 4], color, thickness);
        }
    }
    // Why isn't this function in OpenCV in the first place
    public Point getRectCenter(Rect rect){
        return new Point(rect.tl().x + (rect.width/2.0),rect.tl().y + (rect.height/2.0));
    }

    // Position related methods
    // Height is weird because positive y points down
    public boolean isInHeightWindow(Point point, int maxHeight, int minHeight){
        return point.y > maxHeight && point.y < minHeight;
    }
    public boolean isInWidthWindow(Point point, int leftBound, int rightBound){
        return point.x > leftBound && point.x < rightBound;
    }

    // "corrected" means that the origin is in the center instead of the top left
    public double getCorrectedX(int x){
        return (x - (width /2.0));
    }
    public double getCorrectedY(int y){
        return (y - (height /2.0));
    }

    // Convert pixel positions to angles
    public double getHorizontalAngle(Point point){
        return (getCorrectedX((int) point.x) * pixelsPerHorizontalDegree);
    }
    public double getVerticalAngle(Point point){
        return (getCorrectedY((int) point.y) * pixelsPerVerticalDegree);
    }

    // Scuffed, copied from Utility.java to work with eocv sim
    // Takes an input and checks if it's close enough to the normal value
    public static boolean withinErrorOfValue(double input, double normalValue, double acceptableError){
        double min = normalValue - acceptableError;
        double max = normalValue + acceptableError;

        return (min < input && input < max);
    }
}
