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
}
