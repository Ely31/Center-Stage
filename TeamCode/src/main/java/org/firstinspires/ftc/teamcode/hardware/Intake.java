package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {

    private DcMotor intake;
    private ColorSensor leftSensor;
    private ColorSensor rightSensor;
    private boolean lastInput;
    private boolean intakeToggledStatus;

    public Intake(HardwareMap hwmap) {
        intake = hwmap.get(DcMotor.class, "intake");
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftSensor = hwmap.get(ColorSensor.class,"leftSensor");
        //rightSensor = hwmap.get(ColorSensor.class,"rightSensor");
        lastInput = false;
        intakeToggledStatus = false;
    }

    public void on(){
        intake.setPower(1);
    }
    public void off(){
        intake.setPower(0);
    }
    public void reverse(){
        intake.setPower(-1);
    }

    public void setPower(double power){
        intake.setPower(power);
    }

    public void toggle(boolean input){
        if (input && !lastInput){
            intakeToggledStatus = !intakeToggledStatus;
        }
        if (intakeToggledStatus) on();
        else off();

        lastInput = input;
    }
/*
    // Take the average of the values of both sensors
    public double maxProximity(){
        return Math.max(leftSensor.alpha(), rightSensor.alpha());
    }

    public boolean freightStatus(){
        double maxProximity = maxProximity(); // So we don't make two i2c calls
        return !(325 < maxProximity && maxProximity < 450);
    }

    public boolean isEmpty(){
        double maxProximity = maxProximity(); // So we don't make two i2c calls
        return (325 < maxProximity && maxProximity < 380);
    }
    */
    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("INTAKE");
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Toggled Status", intakeToggledStatus);
    }
}