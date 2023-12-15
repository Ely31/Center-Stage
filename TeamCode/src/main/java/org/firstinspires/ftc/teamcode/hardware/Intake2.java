package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake2 {

    private DcMotor intakeMotor;
    private boolean lastInput;
    private boolean intakeToggledStatus;

    double currentRotations;
    boolean lastPoweredStatus = false;

    double targetPos;

    // Use the formula because that thing will spin hundreds of times in a match and it needs to maintain accurate position
    final double TICKS_PER_REV = ((1+(46.0/11.0)) * 28);

    public Intake2(HardwareMap hwmap) {
        intakeMotor = hwmap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lastInput = false;
        intakeToggledStatus = false;
    }

    public void on(){
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.75);
        lastPoweredStatus = true;
    }
    public void off(){
        if (lastPoweredStatus && !true) {
            // While it's off, straighten the tubing to be vertical so it doesn't mess with anything
            // Use modulo here to ignore the full half rotations and get the important bit, how much it's off from straight
            targetPos = getRotations() - (getRotations() % 0.5);
            intakeMotor.setTargetPosition((int) targetPos);
        }
        // Use rtp here because I'm lazy and this is a use case where it doesn't have to be perfect
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(1);
        lastPoweredStatus = false;
    }
    public void reverse(){
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(-1);
        lastPoweredStatus = true;
    }

    public void setPower(double power){
        intakeMotor.setPower(power);
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

    public double getRotations(){
        return intakeMotor.getCurrentPosition() / TICKS_PER_REV;
    }

    public void update(){
        currentRotations = getRotations();
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("INTAKE");
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Toggled Status", intakeToggledStatus);
        telemetry.addData("Rotations", currentRotations);
        telemetry.addData("Target Pos", targetPos);
        telemetry.addData("Modulused rotations", (currentRotations % 0.5));
    }
}