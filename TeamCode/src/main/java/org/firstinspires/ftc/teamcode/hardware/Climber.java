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
    public static double retractedPos = 0;
    public static double extendedPos = 3.85;

    boolean lastInput = false;
    boolean toggledStatus = false;

    public static double servoHoldPos = 0;
    public static double servoReleasePos = 0.6;

    public static PIDCoefficients coeffs = new PIDCoefficients(3,0.0,0.0);

    public Climber(HardwareMap hwmap){
        climber = new LinearActuator(hwmap, "climber", 13.7, 0.31496);
        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        climber.zero();
        climber.setLimits(retractedPos, extendedPos);

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
        telemetry.addData("release servo pos", climberRelease.getPosition());
        climber.displayDebugInfo(telemetry);
    }
}
