package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.actuators.LinearActuator;

@Config
public class SingleMotorLift {
    LinearActuator liftActuator;

    // Measurements are in inches
    public static double maxHeight = 22;
    public static double minHeight = 0;
    public static double retractedPos = 0;
    public static double extendedPos = 5;

    public static PIDCoefficients coeffs = new PIDCoefficients(0.3,0.08,0.03);
    public static double f = 0.4;

    public SingleMotorLift(HardwareMap hwmap){
        liftActuator = new LinearActuator(hwmap, "leftSpool", 13.7, 5.93);
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
        SingleMotorLift.coeffs = coeffs;
    }

    // Methods
    public void setHeight(double height){
        liftActuator.setDistance(height);
    }
    public double getHeight(){
        return liftActuator.getCurrentDistance();
    }
    public void retract(){
        setHeight(retractedPos);
    }
    public void extend(){
        setHeight(extendedPos);
    }

    public void editExtendedPos(double step){
        extendedPos += step;
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
        liftActuator.update();
    }

    // DANGEROUS!
    public void setRawPowerDangerous(double power){
        liftActuator.setRawPowerDangerous(power);
    }

    public void disalayDebug(Telemetry telemetry){
        liftActuator.displayDebugInfo(telemetry);
    }
}
