package org.firstinspires.ftc.teamcode.vision.workspace;

public class CameraConstants {
    public static final int width = 320, height = 240;
    public static final int midpointX = (int) (width/2.0);
    public static final int midpointY = (int) (height/2.0);
    public static final double aspectRatio = 1.33333;

    // Haven't calibrated the vertical fov yet, migt be totally wrong
    public static final double horizontalFOV = 42, verticalFOV = 42;

    public static final double
        pixelsPerHorizontalDegree = horizontalFOV / width,
        pixelsPerVerticalDegree = verticalFOV / height;
}
