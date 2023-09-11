// Based on OpenFTC's SkystoneDeterminationExample code

package org.firstinspires.ftc.teamcode.vision.old;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SubmatBarcodePipeline extends OpenCvPipeline {

    //An enum to define the TSE position
    public enum TSEPosition
    {
        LEFT,
        MIDDLE,
        RIGHT
    }

    // Some color constants
    public final Scalar BLUE = new Scalar(0, 0, 255);
    public final Scalar GREEN = new Scalar(0, 255, 0);

    // Min and max values for the threshold
    public Scalar lower = new Scalar(0, 65, 40);
    public Scalar upper = new Scalar(255, 115, 130);

    final static int frameWidth = 320;
    final static int thirdWidth = frameWidth/3;
    final static int frameHeight = 240;
    final static int topOfReigions = 100;

    // The core values which define the location and size of the sample regions
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,topOfReigions);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(thirdWidth,topOfReigions);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(thirdWidth*2,topOfReigions);
    static final int REGION_WIDTH = thirdWidth;
    static final int REGION_HEIGHT = frameHeight-topOfReigions;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


    Mat region1_Binary, region2_Binary, region3_Binary;
    Mat YCrCb = new Mat();
    Mat Binary = new Mat();
    int avg1, avg2, avg3;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile TSEPosition position = TSEPosition.LEFT;

    // Converts to YCrCb and then thresholds
    void ThresholdInput(Mat input){
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(YCrCb, lower, upper, Binary);
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        ThresholdInput(firstFrame);

        region1_Binary = Binary.submat(new Rect(region1_pointA, region1_pointB));
        region2_Binary = Binary.submat(new Rect(region2_pointA, region2_pointB));
        region3_Binary = Binary.submat(new Rect(region3_pointA, region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        ThresholdInput(input);
        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel binary Mat, so the value
         * we need is at index 0.
         */
        avg1 = (int) Core.mean(region1_Binary).val[0];
        avg2 = (int) Core.mean(region2_Binary).val[0];
        avg3 = (int) Core.mean(region3_Binary).val[0];

        // Draw rectangles showing the sample regions on the screen.
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                1); // Thickness of the rectangle lines
        // Draw the rest of the regions
        Imgproc.rectangle(input, region2_pointA,region2_pointB,BLUE,1);
        Imgproc.rectangle(input,region3_pointA,region3_pointB,BLUE,1);

        // Find the max of the 3 averages
        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(max == avg1) // Was it from region 1?
        {
            position = TSEPosition.LEFT; // Record our analysis

            // Draw a solid rectangle on top of the chosen region.
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
        }
        else if(max == avg2) // Was it from region 2?
        {
            position = TSEPosition.MIDDLE; // Record our analysis
            Imgproc.rectangle(input,region2_pointA,region2_pointB,GREEN,2);
        }
        else if(max == avg3) // Was it from region 3?
        {
            position = TSEPosition.RIGHT; // Record our analysis
            Imgproc.rectangle(input,region3_pointA,region3_pointB,GREEN,2);
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    // Call this from the OpMode to obtain the latest analysis
    public TSEPosition getAnalysis()
    {
        return position;
    }
}
