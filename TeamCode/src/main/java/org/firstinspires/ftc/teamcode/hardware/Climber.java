package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.actuators.LinearActuator;

@Config
public class Climber {
    LinearActuator climber;
    Servo climberRelease;

    // Measurements are in inches
    public static double maxHeight = 3.75;
    public static double minHeight = 0;
    public static double retractedPos = 0;
    public static double extendedPos = maxHeight;

    boolean lastInput = false;
    boolean toggledStatus = false;

    public static double servoHoldPos = 0;
    public static double servoReleasePos = 1;

    public static PIDCoefficients coeffs = new PIDCoefficients(1,0.0,0.0);

    public Climber(HardwareMap hwmap){
        climber = new LinearActuator(hwmap, "climber", 5.2, 0.31496);
        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        climber.zero();
        climber.setLimits(minHeight, maxHeight);

        climberRelease = hwmap.get(Servo.class, "climberRelease");
        hold();

        setCoefficients(coeffs);
    }

    public void zero(){
        climber.zero();
    }

    public void setCoefficients(PIDCoefficients coeffs){
        climber.setCoefficients(coeffs);
        Climber.coeffs = coeffs;
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

    // Release servo things
    public void hold(){
        climberRelease.setPosition(servoHoldPos);
    }
    public void release(){
        climberRelease.setPosition(servoReleasePos);
    }

    public void disalayDebug(Telemetry telemetry){
        telemetry.addLine("CLIMBER");
        telemetry.addData("state", toggledStatus);
        climber.displayDebugInfo(telemetry);
    }
}
