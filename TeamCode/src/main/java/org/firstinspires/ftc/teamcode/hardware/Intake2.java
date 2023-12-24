package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake2 {

    private DcMotor intakeMotor;

    double currentRotations;
    double currentPower;
    boolean currentPoweredStatus = false;
    boolean lastPoweredStatus = false;

    double targetPos;

    // Use the formula because that thing will spin hundreds of times in a match and it needs to maintain accurate position
    final double TICKS_PER_REV = ((1+(46.0/11.0)) * 28);
    public Intake2(HardwareMap hwmap) {
        intakeMotor = hwmap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void on(){
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.75);
        currentPoweredStatus = true;
    }
    public void off(){
        intakeMotor.setTargetPosition((int) (targetPos*TICKS_PER_REV));
        // Use rtp here because I'm lazy and this is a use case where it doesn't have to be perfect
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(1);
        currentPoweredStatus = false;
    }
    public void reverse(){
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(-1);
        currentPoweredStatus = true;
    }

    public void setPower(double power){
        intakeMotor.setPower(power);
    }
    public double getPower(){return currentPower;}

    public double getRotations(){
        return currentRotations;
    }
    public void setStraightTubingTargetPos(){
        targetPos = getRotations() - (getRotations() % 0.5);
    }

    public void update(){
        currentRotations = intakeMotor.getCurrentPosition() / TICKS_PER_REV;
        currentPower = intakeMotor.getPower();
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("INTAKE");
        telemetry.addData("Intake Power", getPower());
        telemetry.addData("Rotations", currentRotations);
        telemetry.addData("Target Pos", targetPos);
        telemetry.addData("Modulused rotations", (currentRotations % 0.5));
    }
}