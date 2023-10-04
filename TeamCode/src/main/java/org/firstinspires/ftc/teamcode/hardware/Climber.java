package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.actuators.LinearActuator;

@Config
public class Climber {
    LinearActuator liftActuator;

    // Measurements are in inches
    public static double maxHeight = 4;
    public static double minHeight = 0;
    public static double retractedPos = 0;
    public static double extendedPos = 4;

    boolean lastInput = false;
    boolean toggledStatus = false;

    public static PIDCoefficients coeffs = new PIDCoefficients(0.3,0.08,0.03);

    public Climber(HardwareMap hwmap){
        liftActuator = new LinearActuator(hwmap, "lift", 5.2, 0.31496);
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
        Climber.coeffs = coeffs;
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

    public void toggle(boolean input){
        if (input && !lastInput){
            toggledStatus = !toggledStatus;
        }
        if (toggledStatus) extend();
        else retract();

        lastInput = input;
    }

    public void update(){
        liftActuator.update();
    }

    // DANGEROUS!
    public void setRawPowerDangerous(double power){
        liftActuator.setRawPowerDangerous(power);
    }

    public void disalayDebug(Telemetry telemetry){
        telemetry.addLine("CLIMBER");
        telemetry.addData("state", toggledStatus);
        liftActuator.displayDebugInfo(telemetry);
    }
}
