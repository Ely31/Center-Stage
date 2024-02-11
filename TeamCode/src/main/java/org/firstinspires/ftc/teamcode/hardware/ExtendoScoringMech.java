package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class ExtendoScoringMech {
    Lift lift;
    Arm3 arm;
    ExtendoIntake intake;
    PurplePixelPusher ppp;
    // Constructor
    public ExtendoScoringMech(HardwareMap hwmap){
        lift = new Lift(hwmap);
        arm = new Arm3(hwmap);
        intake = new ExtendoIntake(hwmap);
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
        arm.updateSensors(usePixelSensors, false, useBoardSensor);
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
        BUMPING_UP,
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
        SETUP,
        INTAKING,
        MOVING_ARM_DOWN,
        GRABBING,
        SPITTING,
        DONE
    }
    StackGrabbingState stackGrabbingState = StackGrabbingState.SETUP;
    public StackGrabbingState getStackGrabbingState(){
        return stackGrabbingState;
    }
    public void resetStackGrabbingState(){
        stackGrabbingWait.reset();
        stackGrabbingState = StackGrabbingState.SETUP;
    }

    public void grabOffStackAsync(boolean grasping, boolean driving, int startingStackPos){
        switch (stackGrabbingState){
            case SETUP:
                intake.goToStackPosition(startingStackPos);
                stackGrabbingWait.reset();
                stackGrabbingState = StackGrabbingState.INTAKING;
                break;

            case INTAKING:
                retract();
                arm.setStopperState(true);
                intake.on();
                if (stackGrabbingWait.seconds() > 1){
                    stackGrabbingState = StackGrabbingState.MOVING_ARM_DOWN;
                }
                if (grasping){
                    stackGrabbingWait.reset();
                    stackGrabbingState = StackGrabbingState.GRABBING;
                    // Grab 'em
                    arm.setBothGrippersState(true);
                    // Move this up just in case it might be hit
                    intake.goToVertical();
                }
                break;

            case MOVING_ARM_DOWN:
                stackGrabbingWait.reset();
                intake.goToStackPosition(intake.getStackPosition()-1);
                stackGrabbingState = StackGrabbingState.INTAKING;
                break;

            case GRABBING:
                if (stackGrabbingWait.milliseconds() > 350){
                    stackGrabbingWait.reset();
                    arm.preMove();
                    stackGrabbingState = StackGrabbingState.SPITTING;
                }
                break;

            case SPITTING:
                intake.reverse(0.5);
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
                    scoringState = ScoringState.BUMPING_UP;
                }
                break;

            case BUMPING_UP:
                lift.setHeight(height+2);
                if (Utility.withinErrorOfValue(lift.getHeight(), height+2, 0.5)) {
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

    public void setIntakePos(double pos){
        intake.gotoRawPosition(pos);
    }

    // Stuff the ds with info
    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("SCORING MECH");
        telemetry.addData("scoring state", scoringState.name());
        telemetry.addData("scoring wait", scoringWait.milliseconds());
        telemetry.addData("grabbing state", stackGrabbingState.name());
        telemetry.addData("grabbing wait", stackGrabbingWait.milliseconds());
        arm.displayDebug(telemetry);
        lift.disalayDebug(telemetry);
        //ppp.displayDebug(telemetry);
        intake.displayDebug(telemetry);
    }
}
