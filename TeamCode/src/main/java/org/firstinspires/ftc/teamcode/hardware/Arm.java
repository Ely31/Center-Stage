package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class Arm {
    ServoImplEx pivot;
    Servo bottomPixel;
    Servo topPixel;
    ColorSensor bottomPixelSensor;
    ColorSensor topPixelSensor;
    Rev2mDistanceSensor boardSensor;

    boolean bottomState = false; // True is closed, false open
    boolean topState = false;

    // Constants
    public static double pivotMax = 0.85;
    public static double pivotMin = 0.02;
    double pivotIntakingPos = pivotMin;
    double pivotScoringPos = pivotMax;
    public static double pivotPremovePos = 0.36;
    public static double pivotActuationTime = 300;

    public static double pixelClosedPos = 0.88;
    public static double pixelOpenPos = 0.6;
    public static double pixelPosOffset = -0.05;
    public static double pixelActuationTime = 250; // In milliseconds

    public static double sensorThreshold = 6000;

    public Arm(HardwareMap hwmap){
        // Hardwaremap stuff
        pivot = hwmap.get(ServoImplEx.class, "pivot");
        pivot.setDirection(Servo.Direction.REVERSE);
        pivot.setPwmRange(new PwmControl.PwmRange(500,2500));
        bottomPixel = hwmap.get(Servo.class, "bottomPixel");
        topPixel = hwmap.get(Servo.class, "topPixel");
        bottomPixelSensor = hwmap.get(ColorSensor.class, "bottomSensor");
        topPixelSensor = hwmap.get(ColorSensor.class, "topSensor");
        boardSensor = hwmap.get(Rev2mDistanceSensor.class, "boardSensor");

        // Warning: Robot moves on intitialization
        pivotGoToIntake();
        setBothGrippersState(false);
    }

    // Control each
    public void setBottomGripperState(boolean state){
        bottomState = state;
        if (state) bottomPixel.setPosition(pixelClosedPos + pixelPosOffset);
        else bottomPixel.setPosition(pixelOpenPos + pixelPosOffset);
    }
    public boolean getBottomGripperState(){
        return bottomState;
    }

    public void setTopGripperState(boolean state){
        topState = state;
        if (state) topPixel.setPosition(pixelClosedPos);
        else topPixel.setPosition(pixelOpenPos);
    }
    public boolean getTopGripperState(){
        return topState;
    }

    public void setBothGrippersState(boolean state){
        if (state) {
            setBottomGripperState(true); setTopGripperState(true);
            bottomPixel.setPosition(pixelClosedPos + pixelPosOffset);
            topPixel.setPosition(pixelClosedPos);
        }
        else {
            setBottomGripperState(false); setTopGripperState(false);
            bottomPixel.setPosition(pixelOpenPos + pixelPosOffset);
            topPixel.setPosition(pixelClosedPos);
        }
    }
    public boolean getBothGrippersState(){
        // Returns true only if both are closed
        return (getBottomGripperState() && getTopGripperState());
    }

    public void setPivotPos(double pos){
        // Make sure it's a safe move
        double finalPos = Utility.clipValue(pivotMin, pivotMax, pos);
        pivot.setPosition(finalPos);
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

    public boolean pixelIsInBottom(){
        return bottomPixelSensor.alpha() > sensorThreshold;
    }
    public boolean pixelIsInTop(){
        return topPixelSensor.alpha() > sensorThreshold;
    }

    public double getBoardDistance(){
        return boardSensor.getDistance(DistanceUnit.CM);
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addData("Pivot pos", pivot.getPosition());
        telemetry.addData("Bottom pos", bottomPixel.getPosition());
        telemetry.addData("Bottom state", getBottomGripperState());
        telemetry.addData("Bottom sensor val", bottomPixelSensor.alpha());
        telemetry.addData("Pixel in bottom", pixelIsInBottom());
        telemetry.addData("Top pos", topPixel.getPosition());
        telemetry.addData("Top state", getTopGripperState());
        telemetry.addData("Top sensor val", topPixelSensor.alpha());
        telemetry.addData("Pixel in top", pixelIsInTop());
        telemetry.addData("Board distance", getBoardDistance());
    }
}
