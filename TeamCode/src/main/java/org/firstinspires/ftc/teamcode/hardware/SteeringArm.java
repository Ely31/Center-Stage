package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Utility;

import java.util.Arrays;

@Config
public class SteeringArm {
    ServoImplEx pivot;
    Servo bottomPixel;
    Servo topPixel;
    Servo stopper;
    Servo steer;
    ColorSensor bottomPixelSensor;
    ColorSensor topPixelSensor;
    ColorSensor armSensor;
    Rev2mDistanceSensor boardSensor;

    boolean bottomState = false; // True is closed, false open
    boolean topState = false;

    // Constants
    public static double pivotIntakingPos = 0.011;
    public static double pivotScoringPos = 0.938;
    public static double pivotPremovePos = 0.25;
    public static double pivotOffset = 0.002;
    public static double pivotAwayFromBordTime = 300;

    public static double gripperClosedPos = 0.88;
    public static double gripperOpenPos = 0.6;
    public static double gripperPosOffset = -0.05;
    public static double gripperActuationTime = 250; // In milliseconds

    public static double stopperClosedPos = 0.13;
    public static double stopperOpenPos = 0.9;
    boolean stopperState = false;

    public static double bottomSensorThreshold = 800;
    public static double topSensorThreshold = 800;
    public static double armSensorThreshold = 1000;

    double[] lastBottomSensorVals = new double[3];
    double[] lastTopSensorVals = new double[3];
    boolean useRollingPixelSensors = true;

    double lastBoardDistance;
    double lastArmSensorVal;

    double[] lastBoardSensorVals = new double[5];
    boolean useRollingBoardAvg = true;

    ElapsedTime pixelSensorsPollTimer = new ElapsedTime();
    int pixelSensorsPollInterval = 100;
    ElapsedTime armSensorPollTimer = new ElapsedTime();
    int armSensorPollInterval = 200;

    public SteeringArm(HardwareMap hwmap){
        // Hardwaremap stuff
        pivot = hwmap.get(ServoImplEx.class, "pivot");
        pivot.setDirection(Servo.Direction.REVERSE);
        pivot.setPwmRange(new PwmControl.PwmRange(500,2500));
        bottomPixel = hwmap.get(Servo.class, "bottomPixel");
        topPixel = hwmap.get(Servo.class, "topPixel");
        bottomPixelSensor = hwmap.get(ColorSensor.class, "bottomSensor");
        topPixelSensor = hwmap.get(ColorSensor.class, "topSensor");
        boardSensor = hwmap.get(Rev2mDistanceSensor.class, "boardSensor");
        stopper = hwmap.get(Servo.class, "stopper");
        armSensor = hwmap.get(ColorSensor.class, "armSensor");
        steer = hwmap.get(Servo.class, "steer");

        // Warning: Robot moves on intitialization
        pivotGoToIntake();
        setBothGrippersState(false);
        setStopperState(true);
    }

    // Control each
    public void setBottomGripperState(boolean state){
        bottomState = state;
        if (state) bottomPixel.setPosition(gripperClosedPos + gripperPosOffset);
        else bottomPixel.setPosition(gripperOpenPos + gripperPosOffset);
    }
    public boolean getBottomGripperState(){
        return bottomState;
    }

    public void setTopGripperState(boolean state){
        topState = state;
        if (state) topPixel.setPosition(gripperClosedPos);
        else topPixel.setPosition(gripperOpenPos);
    }
    public boolean getTopGripperState(){
        return topState;
    }

    public void setBothGrippersState(boolean state){
        setBottomGripperState(state);
        setTopGripperState(state);
    }
    public boolean getBothGrippersState(){
        // Returns true only if both are closed
        return (getBottomGripperState() && getTopGripperState());
    }

    public void setPivotPos(double pos){
        // Make sure it's a safe move
        double finalPos = Utility.clipValue(pivotIntakingPos, pivotScoringPos, pos);
        pivot.setPosition(finalPos + pivotOffset);
    }
    public double getPivotPos(){
        // Take the pos of the one we didn't offset
        return pivot.getPosition();
    }
    public void pivotGoToIntake(){
        setPivotPos(pivotIntakingPos);
    }
    public void pivotScore(){
        setPivotPos(pivotScoringPos);
    }

    public void preMove(){
        setPivotPos(pivotPremovePos);
    }

    public void setStopperState(boolean state){
        // True for closed, false for open
        if (state) stopper.setPosition(stopperClosedPos);
        else stopper.setPosition(stopperOpenPos);
        stopperState = state;
    }
    public boolean getStopperState() {return stopperState;}

    // Use averages of past values because we get little incorrect blips sometimes
    public boolean pixelIsInBottom(){
        if (useRollingPixelSensors) {
            boolean answer = true;
            for (double val : lastBottomSensorVals) {
                if (val < bottomSensorThreshold) answer = false;
            }
            return answer;
        }
        else return lastBottomSensorVals[0] > bottomSensorThreshold;
    }
    public boolean pixelIsInTop(){
        if (useRollingPixelSensors) {
            boolean answer = true;
            for (double val : lastTopSensorVals) {
                if (val < topSensorThreshold) answer = false;
            }
            return answer;
        }
        else return lastTopSensorVals[0] > topSensorThreshold;
    }
    public boolean armIsDown(){
        return lastArmSensorVal > armSensorThreshold;
    }

    public double getBoardDistance(){
        return lastBoardDistance;
    }
    public double getBoardDistanceRollingAvg(){
        return Arrays.stream(lastBoardSensorVals).average().getAsDouble();
    }

    double angleToServoPos(double angle){
        return 0;
    }

    public void updateSensors(boolean usePixelSensors, boolean useArmSensor, boolean useBoardSensor){
        if (usePixelSensors && pixelSensorsPollTimer.milliseconds() > pixelSensorsPollInterval) {
            if (useRollingPixelSensors) {
                // Shift vals back, this is terrible code and should be done with a for loop
                lastBottomSensorVals[2] = lastBottomSensorVals[1];
                lastBottomSensorVals[1] = lastBottomSensorVals[0];
                lastBottomSensorVals[0] = bottomPixelSensor.alpha();
                // Same thing for top
                lastTopSensorVals[2] = lastTopSensorVals[1];
                lastTopSensorVals[1] = lastTopSensorVals[0];
                lastTopSensorVals[0] = topPixelSensor.alpha();
            }
            else {
                lastTopSensorVals[0] = topPixelSensor.alpha();
                lastBottomSensorVals[0] = bottomPixelSensor.alpha();
            }

            pixelSensorsPollTimer.reset();
        }
        if (useBoardSensor) {
            lastBoardDistance = boardSensor.getDistance(DistanceUnit.CM);
        }
        if (useRollingBoardAvg) {
            for(int i = 4; i >= 0; i--) {
                if (i == 0) lastBoardSensorVals[i] = lastBoardDistance;
                // Shift
                else lastBoardSensorVals[i] = lastBoardSensorVals[i-1];
            }
        }
        if (useArmSensor && armSensorPollTimer.milliseconds() > armSensorPollInterval) {
            lastArmSensorVal = armSensor.alpha();
            armSensorPollTimer.reset();
        }
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("ARM");
        telemetry.addData("Pivot pos", pivot.getPosition());
        telemetry.addData("Bottom pos", bottomPixel.getPosition());
        telemetry.addData("Top pos", topPixel.getPosition());
        telemetry.addData("Bottom gripper state", getBottomGripperState());
        telemetry.addData("Top gripper state", getTopGripperState());
        telemetry.addData("Bottom sensor val", lastBottomSensorVals[0]);
        telemetry.addData("Top sensor val", lastTopSensorVals[0]);
        telemetry.addLine("Bottom sensor past vals");
        for (double val : lastBottomSensorVals){
            telemetry.addData("val", val);
        }
        telemetry.addLine("Top sensor past vals");
        for (double val : lastTopSensorVals){
            telemetry.addData("val", val);
        }
        telemetry.addLine("Board sensor past vals");
        for (double val : lastBoardSensorVals){
            telemetry.addData("val", val);
        }
        telemetry.addData("Pixel in bottom", pixelIsInBottom());
        telemetry.addData("Pixel in top", pixelIsInTop());
        telemetry.addData("Stopper state", getStopperState());
        telemetry.addData("Board distance", getBoardDistance());
        telemetry.addData("Rolling board distance", getBoardDistanceRollingAvg());
        telemetry.addData("Arm sensor val", lastArmSensorVal);
        telemetry.addData("Arm is down", armIsDown());
    }
}
