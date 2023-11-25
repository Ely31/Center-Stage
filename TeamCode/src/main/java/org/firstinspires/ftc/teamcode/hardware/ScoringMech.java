package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class ScoringMech {
    Lift lift;
    Arm arm;
    Intake intake;
    PurplePixelPusher ppp;
    // Constructor
    public ScoringMech(HardwareMap hwmap){
        lift = new Lift(hwmap);
        arm = new Arm(hwmap);
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
        if (armTimerMs > Arm.pivotActuationTime){
            lift.retract();
        }
    }

    public void grabJustForPreload(){
        arm.setBothGrippersState(true);
    }
    public void setPPPState(boolean state) {ppp.setState(state);}


    // ESSENTIAL to call this function every loop
    public void update(){lift.update();}

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

    public enum StackGrabbingState{
        CREEPING,
        GRABBING,
        DONE
    }
    StackGrabbingState stackGrabbingState = StackGrabbingState.CREEPING;
    StackGrabbingState getStackGrabbingState(){
        return stackGrabbingState;
    }

    public void grabOffStackAsync(boolean hasPixels){
        // You have to call updateLift while using this for it to work
        switch (stackGrabbingState){
            case CREEPING:
                arm.setBothGrippersState(false);
                retract();
                intake.on();
                if (hasPixels){
                    stackGrabbingWait.reset();
                    arm.setBothGrippersState(true);
                    stackGrabbingState = StackGrabbingState.GRABBING;
                }
                break;
            case GRABBING:
                if (stackGrabbingWait.milliseconds() > Arm.gripperActuationTime){
                    stackGrabbingWait.reset();
                    arm.preMove();
                    intake.off();
                    stackGrabbingState = StackGrabbingState.DONE;
                }
                break;
        }
    }

    public boolean doneGrabbingOffStack(){
        return stackGrabbingState == StackGrabbingState.DONE;
    }

    public void scoreAsync(double height){
        // You have to call updateLift while using this for it to work
        switch (scoringState){
            case EXTENDING:
                lift.setHeight(height);
                arm.pivotScore();
                arm.setStopperState(false);
                // Move on if the lift is all the way up
                if (Utility.withinErrorOfValue(lift.getHeight(), height, 0.5)) {
                    scoringWait.reset();
                    scoringState = ScoringState.WAITING_FOR_ARM_PIVOT;
                }
                break;

            case WAITING_FOR_ARM_PIVOT:
                if (scoringWait.seconds() > 0.2){ // Wait for the arm to move all the way
                    arm.setBothGrippersState(false); // Drop the pixels
                    scoringWait.reset();
                    scoringState = ScoringState.WAITING_FOR_PIXELS_DROP;
                }
                break;

            case WAITING_FOR_PIXELS_DROP:
                if (scoringWait.seconds() > 0.3){ // Wait for them to fall out
                    scoringWait.reset();
                    scoringState = ScoringState.WAITING_FOR_ARM_RETRACT;
                }
                break;

            case WAITING_FOR_ARM_RETRACT:
                arm.pivotGoToIntake();

                if (scoringWait.milliseconds() > Arm.pivotAwayFromBordTime){
                    scoringWait.reset();
                    lift.retract();
                    scoringState = ScoringState.RETRACTING;
                }
                break;

            case RETRACTING:
                lift.retract();
                arm.setStopperState(true);
                // Move on if the lift is all the way down
                if (Utility.withinErrorOfValue(lift.getHeight(), 0, 1)) {
                    scoringState = ScoringState.DONE; // Finish
                }
                break;

            case DONE:
                break;
        }
    }

    // Have to call this before scoring again to get the state machine to run
    public void resetScoringState(){
        scoringState = ScoringState.EXTENDING;
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
