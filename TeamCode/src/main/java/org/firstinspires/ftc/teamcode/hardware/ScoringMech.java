package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class ScoringMech {
    Lift lift;
    Arm arm;
    Servo bracer;

    public static double bracerExtendedPos = 0.38;
    public static double bracerRetractedPos = 0.7;

    int activeScoringJunction = 2;
    public void setActiveScoringJunction(int level){
        activeScoringJunction = level;
    }
    public int getActiveScoringJunction(){
        return activeScoringJunction;
    }
    // On stackIndex, 0 is the bottom cone and 4 is the very top one on the stack of five
    int stackIndex = 0;
    public int getStackIndex() {
        return stackIndex;
    }
    public void setStackIndex(int stackIndex) {
        this.stackIndex = Utility.clipValue(0,4, stackIndex);
    }

    // Arm pos, then lift pos
    // 0 is on the bottom, 4 is on the very top of the stack
    public static double[] stackPose0 = {Arm.pivotGrabbingPos + 0,     0};
    public static double[] stackPose1 = {Arm.pivotGrabbingPos + 0.00,  1.2};
    public static double[] stackPose2 = {Arm.pivotGrabbingPos + 0.01,  2};
    public static double[] stackPose3 = {Arm.pivotGrabbingPos + 0.02,  2.6};
    public static double[] stackPose4 = {Arm.pivotGrabbingPos + 0.04,  3.2};

    public void setRetractedGrabbingPose(double[] pose){
        arm.setPivotGrabbingPos(pose[0]);
        lift.setRetractedPos(pose[1]);
    }
    public void setRetractedGrabbingPose(int stackIndex){
        switch (stackIndex){
            case 0:
                setRetractedGrabbingPose(stackPose0);
                break;
            case 1:
                setRetractedGrabbingPose(stackPose1);
                break;
            case 2:
                setRetractedGrabbingPose(stackPose2);
                break;
            case 3:
                setRetractedGrabbingPose(stackPose3);
                break;
            case 4:
                setRetractedGrabbingPose(stackPose4);
                break;
        }
    }

    // Constructor
    public ScoringMech(HardwareMap hwmap){
        lift = new Lift(hwmap);
        arm = new Arm(hwmap);
        bracer = hwmap.get(Servo.class, "bracer");
        setStackIndex(0);
        setRetractedGrabbingPose(0);
    }
    public ScoringMech(){}

    // Functions from the arm class
    public void openClaw(){
        arm.openClaw();
    }
    public void closeClaw(){
        arm.closeClaw();
    }
    public void setClawState(boolean state){
        arm.setClawState(state);
    }
    public boolean getClawState(){
        return arm.getClawState();
    }
    public void preMoveV4b(){
        arm.preMoveV4b();
    }
    public double getPivotPos(){
        return arm.getPivotPos();
    }
    public boolean getConeStatus(){
        return arm.coneIsInClaw();
    }

    // Functions from the lift class
    public void editCurrentLiftPos(double step){
        lift.editCurrentPos(activeScoringJunction, step);
    }
    public double getLiftHeight(){
        return lift.getHeight();
    }

    // Bracer functions
    public void extendBracer(){
        bracer.setPosition(bracerExtendedPos);
    }
    public void retractBracer(){
        bracer.setPosition(bracerRetractedPos);
    }

    // Functions that combine arm and lift
    public void scoreHigh(){
        arm.scorePassthrough();
        lift.goToHigh();
    }
    public void scoreMedium(){
        arm.scorePassthrough();
        lift.goToMedium();
    }
    public void scoreLow(){
        arm.scorePassthrough();
        lift.goToLow();
    }
    public void scoreGround(){
        arm.scoreGroundPassthrough();
        lift.goToGround();
    }
    // 0 is ground, 3 is high
    public void scoreOnJunction(int junction){
        switch (junction){
            case 0:
                scoreGround();
                break;
            case 1:
                scoreLow();
                break;
            case 2:
                scoreMedium();
                break;
            case 3:
                scoreHigh();
                break;
        }
        activeScoringJunction = junction;
    }
    public void score(){
        scoreOnJunction(activeScoringJunction);
    }

    public void retract(){
        lift.retract();
        arm.grabPassthrough();
    }
    public void retract(double v4bTimerMs){
        v4bToGrabbingPos();
        if (v4bTimerMs > Arm.pivotActuationTime){
            retractLift();
        }
    }
    public void retractPremoved(double v4bTimerMs){
        preMoveV4b();
        if (v4bTimerMs > Arm.pivotActuationTime){
            retractLift();
        }
    }
    public void zeroLift(){
        lift.zero();
    }

    public void retractLift(){
        lift.retract();
    }
    public void v4bToGrabbingPos(){
        arm.grabPassthrough();
    }

    // ESSENTIAL to call this function every loop
    public void updateLift(){
        lift.update();
    }

    // DANGEROUS!
    public void setRawLiftPowerDangerous(double power){
        lift.setRawPowerDangerous(power);
    }

    // Stuff the ds with telemetry if we want
    public void displayDebug(Telemetry telemetry){
        telemetry.addData("stack index", getStackIndex());
        telemetry.addData("claw state", getClawState());
        arm.displayDebug(telemetry);
        lift.disalayDebug(telemetry);
    }
}
