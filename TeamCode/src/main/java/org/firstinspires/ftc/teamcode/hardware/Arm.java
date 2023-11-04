package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class Arm {
    ServoImplEx pivot;
    Servo bottomPixel;
    Servo topPixel;
    ColorSensor bottomPixelSensor;
    ColorSensor topPixelSensor;

    boolean bottomState = false; // True is closed, false open
    boolean topState = false;

    // Constants
    public static double pivotMax = 0.85;
    public static double pivotMin = 0.0;
    double pivotIntakingPos = pivotMin;
    double pivotScoringPos = pivotMax;
    public static double pivotPremovePos = 0.36;
    public static double pivotActuationTime = 300;

    public static double pixelClosedPos = 0.93;
    public static double pixelOpenPos = 0.45;
    public static double pixelActuationTime = 350; // In milliseconds

    public static double sensorThreshold = 800;

    public Arm(HardwareMap hwmap){
        // Hardwaremap stuff
        pivot = hwmap.get(ServoImplEx.class, "pivot");
        pivot.setDirection(Servo.Direction.REVERSE);
        pivot.setPwmRange(new PwmControl.PwmRange(500,2500));
        bottomPixel = hwmap.get(Servo.class, "bottomPixel");
        topPixel = hwmap.get(Servo.class, "topPixel");
        bottomPixelSensor = hwmap.get(ColorSensor.class, "bottomSensor");
        topPixelSensor = hwmap.get(ColorSensor.class, "topSensor");

        // Warning: Robot moves on intitialization
        pivotGoToIntake();
        setBothGrippers(false);
    }

    // Control each
    public void setBottomGripperState(boolean state){
        bottomState = state;
        if (state) bottomPixel.setPosition(pixelClosedPos);
        else bottomPixel.setPosition(pixelOpenPos);
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

    public void setBothGrippers(boolean state){
        if (state) {
            setBottomGripperState(true); setTopGripperState(true);
            bottomPixel.setPosition(pixelClosedPos);
            topPixel.setPosition(pixelClosedPos);
        }
        else {
            setBottomGripperState(false); setTopGripperState(false);
            bottomPixel.setPosition(pixelOpenPos);
            topPixel.setPosition(pixelClosedPos);
        }
    }
    public boolean getBothGrippers(){
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

    public void displayDebug(Telemetry telemetry){
        telemetry.addData("Pivot pos", pivot.getPosition());
        telemetry.addData("Claw pos", bottomPixel.getPosition());
        telemetry.addData("Claw closed", getBottomGripperState());
        telemetry.addData("Claw sensor val", bottomPixelSensor.alpha());
        telemetry.addData("Cone in claw", pixelIsInBottom());
    }
}
