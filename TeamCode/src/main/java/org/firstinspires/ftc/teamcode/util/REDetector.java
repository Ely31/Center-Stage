package org.firstinspires.ftc.teamcode.util;

public class REDetector {

    boolean prevInput = false;

    public boolean status(boolean input){
        boolean ans = (input && !prevInput);
        prevInput = input;
        return ans;
    }
}
