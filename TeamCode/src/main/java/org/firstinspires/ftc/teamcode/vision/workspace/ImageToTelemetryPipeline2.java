package org.firstinspires.ftc.teamcode.vision.workspace;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ImageToTelemetryPipeline2 extends OpenCvPipeline {
    Mat shrunkMat = new Mat();
    int imageWidth = 35;
    public int imageHeight = (int) (imageWidth / CameraConstants.aspectRatio);

    @Override
    public Mat processFrame(Mat input) {
        // Scale the image down so we can print each pixel to telemetry and they all fit on the screen without wrapping
        Imgproc.resize(input, shrunkMat, new Size(imageWidth, imageHeight), Imgproc.INTER_AREA);

        return shrunkMat;
    }

    // The important method
    public String imagePixelToTelemetryPixel(int row, int col){
        return (
                "<font color =#"
                + Integer.toHexString((int) shrunkMat.get(row, col)[0])
                + Integer.toHexString((int) shrunkMat.get(row, col)[1])
                + Integer.toHexString((int) shrunkMat.get(row, col)[2])
                +">█</font>"
        );
    }
    // Stick a bunch of pixels together in a row
    public String rowToDisplayPixels(int row){
        StringBuilder outputString = new StringBuilder();
        // Iterate through every pixel in the row and tack it on to the output string
        // The ide and stack overflow told me to use StringBuilder so ok
        for(int i=0; i < imageWidth; i++){
            outputString.append(imagePixelToTelemetryPixel(row,i));
        }
        return outputString.toString();
    }
    // Stick a bunch of rows together
    public void toTelemetry(Telemetry telemetry){
        // Write all the rows of pixels out in telemetry
        for (int i = 0; i < imageHeight; i++) {
            telemetry.addLine(rowToDisplayPixels(i));
        }
    }
}
