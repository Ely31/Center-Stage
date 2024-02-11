package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class ExtendoIntake {

    private DcMotor intakeMotor;
    private Servo intakeArmServo;
    private boolean lastInput;
    private boolean intakeToggledStatus;

    public static double verticalPos = 0.81;
    public static double groundPos = 0.3;
    public static double aboveStackPos = 0.5;
    public static double servoOffset = 0;

    private int stackPosition = 0;
    public int getStackPosition(){return stackPosition;}
    private double stackpositions[] = new double[]{0.335,0.36,0.38,0.42,0.436, aboveStackPos};

    public ExtendoIntake(HardwareMap hwmap) {
        intakeMotor = hwmap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeArmServo = hwmap.get(Servo.class, "intakeArmServo");
        intakeArmServo.setPosition(verticalPos);
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
    public boolean getToggledStatus(){return intakeToggledStatus;}

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
        stackPosition = Utility.clipValue(0, stackpositions.length-1, position);
        // Linearly interpolate here or have an array of predefined positions?
        intakeArmServo.setPosition(stackpositions[stackPosition] + servoOffset);
    }
    public void gotoRawPosition(double pos){
        intakeArmServo.setPosition(pos);
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("INTAKE");
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Toggled Status", intakeToggledStatus);
        telemetry.addData("Stack position", stackPosition);
        telemetry.addData("Servo pos", intakeArmServo.getPosition());
    }
}