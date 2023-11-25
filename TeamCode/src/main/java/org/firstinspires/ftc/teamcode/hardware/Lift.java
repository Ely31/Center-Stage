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
    LinearActuator liftActuator;

    // Measurements are in inches
    public static double maxHeight = 29;
    public static double minHeight = 0;
    public static double retractedPos = 0;
    public static double extendedPos = 7;

    public static PIDCoefficients coeffs = new PIDCoefficients(0.5,0.04,0.04);
    public static double f = 0.0;

    boolean state = false; // True for extended
    public boolean getState() {return state;}

    double targetHeight;

    public Lift(HardwareMap hwmap){
        liftActuator = new LinearActuator(hwmap, "lift", 13.7, 5.93);
        liftActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftActuator.zero();
        liftActuator.setLimits(minHeight, maxHeight);

        setCoefficients(coeffs);
    }

    public void zero(){
        liftActuator.zero();
    }

    public void setCoefficients(PIDCoefficients coeffs){
        liftActuator.setCoefficients(coeffs);
        liftActuator.setfCoefficient(f);
        Lift.coeffs = coeffs;
    }

    // Methods
    public void setHeight(double height){
        liftActuator.setDistance(height);
    }
    public double getHeight(){
        return liftActuator.getCurrentDistance();
    }
    public void retract(){
        state = false;
    }
    public void extend(){
        state = true;
    }

    public void editExtendedPos(double step){
        extendedPos += step;
        extendedPos = Utility.clipValue(minHeight, maxHeight, extendedPos);
    }
    public void setExtendedPos(double height){
        extendedPos = height;
    }
    public double getExtendedPos(){
        return extendedPos;
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
        if (state) targetHeight = extendedPos; else targetHeight = retractedPos;
        setHeight(targetHeight);
        liftActuator.update();
    }

    // DANGEROUS!
    public void setRawPowerDangerous(double power){
        liftActuator.setRawPowerDangerous(power);
    }

    public void disalayDebug(Telemetry telemetry){
        telemetry.addLine("LIFT");
        telemetry.addData("state", state);
        liftActuator.displayDebugInfo(telemetry);
    }
}
