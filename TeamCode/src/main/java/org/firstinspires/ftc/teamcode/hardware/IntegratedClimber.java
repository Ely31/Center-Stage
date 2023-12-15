package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.actuators.LinearActuator;

@Config
public class IntegratedClimber {
    LinearActuator climber;

    // Measurements are in inches
    public static double retractedPos = 0;
    public static double extendedPos = 3.85;

    boolean lastInput = false;
    boolean toggledStatus = false;

    public static PIDCoefficients coeffs = new PIDCoefficients(3,0.1,0.2);

    public IntegratedClimber(HardwareMap hwmap){
        climber = new LinearActuator(hwmap, "climber", 50.9, 0.31496);
        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        climber.zero();
        climber.setLimits(retractedPos, extendedPos);

        setCoefficients(coeffs);
    }

    public void zero(){
        climber.zero();
    }

    public void setCoefficients(PIDCoefficients coeffs){
        climber.setCoefficients(coeffs);
        IntegratedClimber.coeffs = coeffs;
    }

    // Methods
    public void setHeight(double height){
        climber.setDistance(height);
    }
    public double getHeight(){
        return climber.getCurrentDistance();
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
        climber.update();
    }

    // DANGEROUS!
    public void setRawPowerDangerous(double power){
        climber.setRawPowerDangerous(power);
    }

    public void disalayDebug(Telemetry telemetry){
        telemetry.addLine("CLIMBER");
        telemetry.addData("state", toggledStatus);
        climber.displayDebugInfo(telemetry);
    }
}
