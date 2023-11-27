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
    // Used in teleop whenever we extend because the arm servo doesn't have positional feedback
    public void forceToggleOff(){
        intakeToggledStatus = false;
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("INTAKE");
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Toggled Status", intakeToggledStatus);
    }
}