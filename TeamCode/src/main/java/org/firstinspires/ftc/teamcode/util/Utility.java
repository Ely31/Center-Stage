package org.firstinspires.ftc.teamcode.util;

public class Utility {
    // Takes an input and if it is outside the range, make it inside the range
    public static double clipValue(double min, double max, double input){
        return Math.max(min, Math.min(input, max)); // Copied from stack overflow
    }
    public static int clipValue(int min, int max, int input){
        return Math.max(min, Math.min(input, max)); // Copied from stack overflow
    }
    // Takes an input and checks if it's close enough to the normal value
    public static boolean withinErrorOfValue(double input, double normalValue, double acceptableError){
        double min = normalValue - acceptableError;
        double max = normalValue + acceptableError;

        return (min < input && input < max);
    }

    public static String generateTelemetryTrackbar(double minval, double maxval, double input, double resolution){
        double percentage = (maxval-minval)/(input-minval);
        int segmentToLight = (int) (resolution*percentage);
        StringBuilder builder = new StringBuilder();
        for (int i=0; i < resolution; i++){
            if (i == segmentToLight) builder.append("█");
            else builder.append("-");
        }
        return builder.toString();
    }
}
