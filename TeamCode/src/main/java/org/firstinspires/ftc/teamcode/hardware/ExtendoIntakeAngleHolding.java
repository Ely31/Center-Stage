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
public class ExtendoIntakeAngleHolding {

    private DcMotorEx intakeMotor;
    private Servo intakeArmServo;
    private boolean lastInput;
    private boolean intakeToggledStatus;

    public static double verticalPos = 0.81;
    public static double groundPos = 0.3;
    public static double aboveStackPos = 0.49;
    public static double servoOffset = 0;

    final double TICKS_PER_REV = ((1+(46.0/11.0)) * 28);
    int targetRotations;

    private int stackPosition = 5;
    public int getStackPosition(){return stackPosition;}
    private double stackpositions[] = new double[]{0.34,0.36,0.38,0.42,0.44, aboveStackPos};

    ElapsedTime jamTimer = new ElapsedTime();
    ElapsedTime startCorrectingAngleTimer = new ElapsedTime();
    int antiJamSpittingTime = 150;
    double lastCurrent = 0;

    public ExtendoIntakeAngleHolding(HardwareMap hwmap) {
        intakeMotor = hwmap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmServo = hwmap.get(Servo.class, "intakeArmServo");
        intakeArmServo.setPosition(verticalPos);
        lastInput = false;
        intakeToggledStatus = false;
        jamTimer.reset();
    }

    public void on(){
        this.on(false, 1);
    }
    public void on(boolean antiJam, double power){
        startCorrectingAngleTimer.reset();
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // If the current trips the limit, spit out for a little bit
        if (antiJam){
            updateCurrent();
            if (lastCurrent > 6.2 && jamTimer.milliseconds() > antiJamSpittingTime + 1000){
                jamTimer.reset();
            }
            if (jamTimer.milliseconds() < antiJamSpittingTime){
                reverse();
            } else intakeMotor.setPower(power);
        }
        else {
            intakeMotor.setPower(power);
        }
    }

    public void off(){
        // If it was just shut off recently, let it coast and stop moving before correcting the position
        if (startCorrectingAngleTimer.seconds() < 0.1){
            intakeMotor.setPower(0);
            targetRotations = (int) ((intakeMotor.getCurrentPosition()/TICKS_PER_REV) - ((intakeMotor.getCurrentPosition()/TICKS_PER_REV) % 0.5));
        }
        else {
            intakeMotor.setTargetPosition((int) (targetRotations*TICKS_PER_REV));
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setPower(1);
        }
    }
    public void reverse(double speed){
        startCorrectingAngleTimer.reset();
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(-Math.abs(speed));
    }
    public void reverse(){
        this.reverse(1);
    }

    public void setPower(double power){
        intakeMotor.setPower(power);
    }

    public void toggle(boolean input, boolean antiJam, double power){
        if (input && !lastInput){
            intakeToggledStatus = !intakeToggledStatus;
        }
        if (intakeToggledStatus) on(antiJam, power);
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
        this.toggle(input, false, 1);
    }
    public void updateCurrent(){
        lastCurrent = intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
    // Used in teleop to stop it when the arm isn't there
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
        telemetry.addData("Target rotations", targetRotations);
        telemetry.addData("angle correcting timer seconds", startCorrectingAngleTimer.seconds());
    }
}