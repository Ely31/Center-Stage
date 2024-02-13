package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class ExtendoIntake {

    private DcMotorEx intakeMotor;
    private Servo intakeArmServo;
    private boolean lastInput;
    private boolean intakeToggledStatus;

    public static double verticalPos = 0.81;
    public static double groundPos = 0.3;
    public static double aboveStackPos = 0.49;
    public static double servoOffset = 0;

    private int stackPosition = 5;
    public int getStackPosition(){return stackPosition;}
    private double stackpositions[] = new double[]{0.335,0.36,0.38,0.42,0.44, aboveStackPos};

    ElapsedTime jamTimer = new ElapsedTime();
    int antiJamSpittingTime = 150;
    double lastCurrent = 0;

    public ExtendoIntake(HardwareMap hwmap) {
        intakeMotor = hwmap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeArmServo = hwmap.get(Servo.class, "intakeArmServo");
        intakeArmServo.setPosition(verticalPos);
        lastInput = false;
        intakeToggledStatus = false;
        jamTimer.reset();
    }

    public void on(){
        this.on(false);
    }
    public void on(boolean antiJam){
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // If the current trips the limit, spit out for a little bit
        if (antiJam){
            updateCurrent();
            if (lastCurrent > 6.2 && jamTimer.milliseconds() > antiJamSpittingTime + 1000){
                jamTimer.reset();
            }
            if (jamTimer.milliseconds() < antiJamSpittingTime){
                reverse();
            } else intakeMotor.setPower(0.75);
        }
        else {
            intakeMotor.setPower(0.75);
        }
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

    public void toggle(boolean input, boolean antiJam){
        if (input && !lastInput){
            intakeToggledStatus = !intakeToggledStatus;
        }
        if (intakeToggledStatus) on(antiJam);
        else {
            if (antiJam) {
                if (!(jamTimer.milliseconds() < antiJamSpittingTime)) {
                    off();
                }
            } else off();
        }

        lastInput = input;
    }
    public void toggle(boolean input){
        this.toggle(input, false);
    }
    public void updateCurrent(){
        lastCurrent = intakeMotor.getCurrent(CurrentUnit.AMPS);
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
        // Look up the position in the array and go there
        intakeArmServo.setPosition(stackpositions[stackPosition] + servoOffset);
    }
    public void gotoRawPosition(double pos){
        intakeArmServo.setPosition(pos);
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("INTAKE");
        telemetry.addData("Toggled Status", intakeToggledStatus);
        telemetry.addData("Current", lastCurrent);
        telemetry.addData("Jam timer", jamTimer.milliseconds());
        telemetry.addData("Stack position", stackPosition);
        telemetry.addData("Servo pos", intakeArmServo.getPosition());
    }
}