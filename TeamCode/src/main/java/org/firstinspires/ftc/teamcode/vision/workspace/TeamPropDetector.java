// Based on OpenFTC's SkystoneDeterminationExample code
// And now based off our ff code

package org.firstinspires.ftc.teamcode.vision.workspace;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDetector extends OpenCvPipeline {

    // Don't use this first constructor unless it's for testing
    public TeamPropDetector(){
        lower = orangeLower;
        upper = orangeUpper;
    }
    public TeamPropDetector(boolean isRedAlliance){
        if (isRedAlliance){
            lower = orangeLower;
            upper = orangeUpper;
        } else {
            lower = blueLower;
            upper = blueUpper;
        }
    }

    // 0 is left, 1 middle, 2 right
    public int propPosition;

    // Some color constants
    public final Scalar BLUE = new Scalar(0, 0, 255);
    public final Scalar GREEN = new Scalar(0, 255, 0);
    public final Scalar WHITE = new Scalar(255,255,255);

    // Min and max values for the threshold
    public Scalar orangeLower = new Scalar(0, 65, 40);
    public Scalar orangeUpper = new Scalar(255, 115, 130);
    public Scalar blueLower = new Scalar(0, 65, 40);
    public Scalar blueUpper = new Scalar(255, 115, 130);
    public Scalar lower;
    public Scalar upper;

    // Variables that determine the placement of the boxes
    final static int frameWidth = 320;
    final static int center = frameWidth/2;
    final static int frameHeight = 240;
    final static int topOfSides = 120;
    final static int topOfMiddle = 110;
    final static int sidesSpan = 100;
    static final int REGION_WIDTH = 30;
    static final int REGION_HEIGHT = 30;

    // The core values which define the location and size of the sample regions
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(center - sidesSpan - (int)(REGION_WIDTH/2),topOfSides);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(center - (int)(REGION_WIDTH/2),topOfMiddle);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(center + sidesSpan - (int)(REGION_WIDTH/2),topOfSides);

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
        // Write the average brightness next to it
        Imgproc.putText(
                input,
                String.valueOf(avg1),
                region1_pointA,
                1,
                2,
                WHITE
                );
        // Draw the rest of the regions and text
        Imgproc.rectangle(input, region2_pointA,region2_pointB,BLUE,1);
        Imgproc.putText(input, String.valueOf(avg2), region2_pointA, 1, 2, WHITE);
        Imgproc.rectangle(input,region3_pointA,region3_pointB,BLUE,1);
        Imgproc.putText(input, String.valueOf(avg3), region3_pointA, 1, 2, WHITE);

        // Find the max of the 3 averages
        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(max == avg1) // Was it from region 1?
        {
            propPosition = 1; // Record our analysis

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
            propPosition = 2; // Record our analysis
            Imgproc.rectangle(input,region2_pointA,region2_pointB,GREEN,2);
        }
        else {
            propPosition = 3; // Record our analysis
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
    public int getAnalysis()
    {
        return propPosition;
    }
}
