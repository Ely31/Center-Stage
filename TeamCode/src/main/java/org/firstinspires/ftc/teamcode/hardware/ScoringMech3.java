package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class ScoringMech3 {
    Lift lift;
    Arm3 arm;
    Intake intake;
    PurplePixelPusher ppp;
    // Constructor
    public ScoringMech3(HardwareMap hwmap){
        lift = new Lift(hwmap);
        arm = new Arm3(hwmap);
        intake = new Intake(hwmap);
        ppp = new PurplePixelPusher(hwmap);
    }

    public void score(){
        lift.extend();
        arm.pivotScore();
        // More stuff to do
    }

    public void retract(){
        lift.retract();
        arm.pivotGoToIntake();
    }
    public void retract(double armTimerMs){
        arm.pivotGoToIntake();
        if (armTimerMs > Arm3.pivotAwayFromBordTime){
            lift.retract();
        }
    }

    public void grabJustForPreload(){arm.setBothGrippersState(true);}
    public void setPPPState(boolean state) {ppp.setState(state);}
    public void premove(){arm.preMove();}

    // ESSENTIAL to call this function every loop
    public void update(boolean usePixelSensors, boolean useBoardSensor) {
        lift.update();
        arm.update(usePixelSensors, false, useBoardSensor);
    }

    public boolean hasBothPixels(){
        return arm.pixelIsInBottom() && arm.pixelIsInTop();
    }

    ElapsedTime scoringWait = new ElapsedTime();
    ElapsedTime stackGrabbingWait = new ElapsedTime();

    public enum ScoringState{
        EXTENDING,
        WAITING_FOR_ARM_PIVOT,
        WAITING_FOR_PIXELS_DROP,
        WAITING_FOR_ARM_RETRACT,
        RETRACTING,
        DONE
    }
    ScoringState scoringState = ScoringState.EXTENDING;
    public ScoringState getScoringState() {
        return scoringState;
    }
    // Have to call this before scoring again to get the state machine to run
    public void resetScoringState(){
        scoringState = ScoringState.EXTENDING;
    }

    public enum StackGrabbingState{
        KNOCKING,
        INTAKING,
        GRABBING,
        SPITTING,
        DONE
    }
    StackGrabbingState stackGrabbingState = StackGrabbingState.KNOCKING;
    public StackGrabbingState getStackGrabbingState(){
        return stackGrabbingState;
    }
    public void resetStackGrabbingState(){
        stackGrabbingWait.reset();
        stackGrabbingState = StackGrabbingState.KNOCKING;
    }

    public void grabOffStackAsync(boolean grasping, boolean knockOverStack){
        switch (stackGrabbingState){
            case KNOCKING:
                arm.setBothGrippersState(false);
                retract();
                intake.reverse(0.5);
                // Skip this state if we want, like when we're going for the second cycle
                if (stackGrabbingWait.seconds() > 0.8 || !knockOverStack){
                    stackGrabbingWait.reset();
                    stackGrabbingState = StackGrabbingState.INTAKING;
                }
                break;

            case INTAKING:
                retract();
                arm.setStopperState(true);
                intake.on();
                if (grasping){
                    stackGrabbingWait.reset();
                    stackGrabbingState = StackGrabbingState.GRABBING;
                    // Grab 'em
                    arm.setBothGrippersState(true);
                }
                break;

            case GRABBING:
                if (stackGrabbingWait.milliseconds() > 350){
                    stackGrabbingWait.reset();
                    arm.preMove();
                    stackGrabbingState = StackGrabbingState.SPITTING;
                }
                break;

            case SPITTING:
                intake.reverse();
                if (stackGrabbingWait.seconds() > 0.5){
                    stackGrabbingState = StackGrabbingState.DONE;
                }
                break;

            case DONE:
                intake.off();
                break;
        }
    }

    public boolean doneGrabbingOffStack(){
        return stackGrabbingState == StackGrabbingState.DONE;
    }

    public void scoreAsync(double height){
        switch (scoringState){
            case EXTENDING:
                lift.setHeight(height);
                arm.pivotScore();
                arm.setStopperState(false);
                arm.setBothGrippersState(true);
                // Move on if the lift is all the way up
                if (Utility.withinErrorOfValue(lift.getHeight(), height, 0.5)) {
                    scoringWait.reset();
                    scoringState = ScoringState.WAITING_FOR_ARM_PIVOT;
                }
                break;

            case WAITING_FOR_ARM_PIVOT:
                if (scoringWait.seconds() > 1) { // Wait for the arm to move all the way
                    arm.setBothGrippersState(false); // Drop the pixels
                    scoringWait.reset();
                    scoringState = ScoringState.WAITING_FOR_PIXELS_DROP;
                }
                break;

            case WAITING_FOR_PIXELS_DROP:
                if (scoringWait.seconds() > 0.5){ // Wait for them to fall out
                    scoringWait.reset();
                    scoringState = ScoringState.WAITING_FOR_ARM_RETRACT;
                }
                break;

            case WAITING_FOR_ARM_RETRACT:
                arm.pivotGoToIntake();

                if (scoringWait.milliseconds() > Arm3.pivotAwayFromBordTime){
                    scoringWait.reset();
                    lift.retract();
                    scoringState = ScoringState.RETRACTING;
                }
                break;

            case RETRACTING:
                lift.retract();
                // Move on if the lift is all the way down
                if (Utility.withinErrorOfValue(lift.getHeight(), 0, 1)) {
                    scoringState = ScoringState.DONE; // Finish
                }
                break;

            case DONE:
                break;
        }
    }

    public boolean liftIsGoingDown(){
        return scoringState == ScoringState.WAITING_FOR_ARM_RETRACT;
    }
    public boolean liftIsMostlyDown(){
        return scoringState == ScoringState.RETRACTING;
    }

    // Stuff the ds with info
    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("SCORING MECH");
        telemetry.addData("scoring state", scoringState.name());
        telemetry.addData("grabbing state", stackGrabbingState.name());
        arm.displayDebug(telemetry);
        lift.disalayDebug(telemetry);
        ppp.displayDebug(telemetry);
        intake.displayDebug(telemetry);
    }
}
