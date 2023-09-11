package org.firstinspires.ftc.teamcode.vision.workspace;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ImageToTelemetryPipeline extends OpenCvPipeline {

    Telemetry telemetry;
    public ImageToTelemetryPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public ImageToTelemetryPipeline(){}

    Mat shrunkMat = new Mat();

    int imageWidth = 35;
    public int imageHeight = (int) (imageWidth / CameraConstants.aspectRatio);

    public static boolean telemetryEnabled = false;

    String outPutString;

    @Override
    public Mat processFrame(Mat input) {
        // Scale the image down so we can print each pixel to telemetry and they all fit
        Imgproc.resize(input, shrunkMat, new Size(imageWidth, imageHeight), Imgproc.INTER_AREA);

        if(telemetryEnabled) {
            outPutString = rowToDisplayPixels(0);
            telemetry.addLine(outPutString);
            telemetry.update();
        }

        return shrunkMat;
    }

    public int[] pixelToRGB(int row, int col){
        int[] output = new int[3];
        output[0] = (int) shrunkMat.get(row, col)[0];
        output[1] = (int) shrunkMat.get(row, col)[1];
        output[2] = (int) shrunkMat.get(row, col)[2];

        return output;
    }
    public String RGBToHex(int[] input){
        String redString = Integer.toHexString(input[0]);
        String greenString = Integer.toHexString(input[1]);
        String blueString = Integer.toHexString(input[2]);

        return redString + greenString + blueString;
    }
    public String pixelToHex(int row, int col){
        return RGBToHex(pixelToRGB(row,col));
    }
    public String hexToDisplayPixel(String color){
        return "<font color =#" + color + ">█</font>";
    }

    // The method that ties all of this together
    public String pixelToDisplayPixel(int row, int col){
        return hexToDisplayPixel(pixelToHex(row, col));
    }

    public String rowToDisplayPixels(int row){
        StringBuilder outputString = new StringBuilder();
        // Iterate through every pixel in the row and tack it on to the output string
        // The ide and stack overflow told me to use StringBuilder so ok
        for(int i=0; i < imageWidth; i++){
            outputString.append(pixelToDisplayPixel(row, i));
        }
        return outputString.toString();
    }

}
