package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class FixedIntake {

    private DcMotor intakeMotor;
    private boolean lastInput;
    private boolean intakeToggledStatus;

    public FixedIntake(HardwareMap hwmap) {
        intakeMotor = hwmap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lastInput = false;
        intakeToggledStatus = false;
    }

    public void on(){
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.75);
    }
    public void off(){
        intakeMotor.setPower(0);
    }
    public void reverse(double speed){
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(-Math.abs(speed));
    }
    public void reverse(){
        this.reverse(1);
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

    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("INTAKE");
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Toggled Status", intakeToggledStatus);
    }
}