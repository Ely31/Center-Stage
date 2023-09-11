package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.actuators.LinearActuator;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class Lift {
    LinearActuator left;
    LinearActuator right;

    // Measurements are in inches
    public static double maxHeight = 22;
    public static double minHeight = 0;

    public static double retractedPos = 0;
    public static double groundPos = 0;
    public static double lowPos = 0;
    public static double mediumPos = 10;
    public static double highPos = 19;

    public static PIDCoefficients coeffs = new PIDCoefficients(0.3,0.08,0.03);
    public static double f = 0.4;

    public Lift(HardwareMap hwmap){
        left = new LinearActuator(hwmap, "leftSpool", 13.7, 5.93);
        right = new LinearActuator(hwmap, "rightSpool", 13.7, 5.93);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left.zero();
        right.zero();
        left.setLimits(minHeight, maxHeight);
        right.setLimits(minHeight, maxHeight);
        right.setReversed();

        setCoefficients(coeffs);
    }

    public void zero(){
        left.zero();
        right.zero();
    }

    public void setCoefficients(PIDCoefficients coeffs){
        left.setCoefficients(coeffs);
        right.setCoefficients(coeffs);
        Lift.coeffs = coeffs;
        left.setfCoefficient(f);
        right.setfCoefficient(f);
    }

    // Methods
    public void setHeight(double height){
        left.setDistance(height);
        right.setDistance(height);
    }
    public double getHeight(){
        return ((left.getCurrentDistance() + right.getCurrentDistance()) / 2);
    }
    public void retract(){
        setHeight(retractedPos);
    }
    public void goToGround(){
        setHeight(groundPos);
    }
    public void goToLow(){
        setHeight(lowPos);
    }
    public void goToMedium(){
        setHeight(mediumPos);
    }
    public void goToHigh(){
        setHeight(highPos);
    }

    public void goToJunction(int junction){
        switch (junction){
            case 0:
                goToGround();
                break;
            case 1:
                goToLow();
                break;
            case 2:
                goToMedium();
                break;
            case 3:
                goToHigh();
                break;
        }
    }

    public void editCurrentPos(int posToedit, double step){
        switch (posToedit){
            case 0:
                groundPos += step;
                break;
            case 1:
                lowPos += step;
                break;
            case 2:
                mediumPos += step;
                break;
            case 3:
                highPos += step;
                break;
        }
    }

    public void editRetractedPos(double step){
        retractedPos += step;
    }
    public void setRetractedPos(double pos) {
        retractedPos = pos;
    }
    public double getRetractedPos(){
        return retractedPos;
    }
    public void resetRetractedPos(){
        retractedPos = 0;
    }

    public void update(){
        left.update();
        right.update();
    }

    // DANGEROUS!
    public void setRawPowerDangerous(double power){
        left.setRawPowerDangerous(power);
        right.setRawPowerDangerous(power);
    }

    public void disalayDebug(Telemetry telemetry){
        left.displayDebugInfo(telemetry);
    }
}
