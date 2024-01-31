package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AdjustableIntake {

    private DcMotor intakeMotor;
    private Servo intakeArmServo;
    private boolean lastInput;
    private boolean intakeToggledStatus;

    public static double verticalPos = 0;
    public static double groundPos = 0.6;
    public static double aboveStackPos = 0.4;

    private int stackPosition = 4;
    public int getStackPosition(){return stackPosition;}

    public AdjustableIntake(HardwareMap hwmap) {
        intakeMotor = hwmap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmServo = hwmap.get(Servo.class, "intakeArmServo");
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

    public void goToVertical(){
        intakeArmServo.setPosition(verticalPos);
    }
    public void goToGround(){
        intakeArmServo.setPosition(groundPos);
    }
    public void goAboveStack(){
        intakeArmServo.setPosition(aboveStackPos);
    }
    public void goToStackPosition(int position){
        stackPosition = position;
        // Linearly interpolate here or have an array of predefined positions?
        intakeArmServo.setPosition(0);
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("INTAKE");
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Toggled Status", intakeToggledStatus);
    }
}