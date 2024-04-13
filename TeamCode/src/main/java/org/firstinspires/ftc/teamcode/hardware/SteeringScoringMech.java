package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class SteeringScoringMech {
    Lift lift;
    SteeringArm arm;
    ExtendoIntakeAngleHolding intake;
    // Constructor
    public SteeringScoringMech(HardwareMap hwmap){
        lift = new Lift(hwmap);
        arm = new SteeringArm(hwmap);
        intake = new ExtendoIntakeAngleHolding(hwmap);
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
    public void premove(){arm.preMove();}

    // ESSENTIAL to call this function every loop
    public void update(boolean usePixelSensors, boolean useBoardSensor) {
        lift.update();
        arm.updateSensors(usePixelSensors, false, useBoardSensor);
    }

    public boolean hasBothPixels(){
        return arm.pixelIsInBottom() && arm.pixelIsInTop();
    }
    public boolean hasAPixel(){
        return arm.pixelIsInBottom() || arm.pixelIsInTop();
    }

    ElapsedTime scoringWait = new ElapsedTime();
    ElapsedTime stackGrabbingWait = new ElapsedTime();

    public enum ScoringState{
        SETUP,
        EXTENDING,
        WAITING_FOR_ARM_PIVOT,
        WAITING_FOR_PIXELS_DROP,
        BUMPING_UP,
        WAITING_FOR_ARM_RETRACT,
        RETRACTING,
        DONE
    }
    ScoringState scoringState = ScoringState.SETUP;
    public ScoringState getScoringState() {
        return scoringState;
    }
    // Have to call this before scoring again to get the state machine to run
    public void resetScoringState(){
        scoringState = ScoringState.SETUP;
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

    public void grabOffStackAsync(boolean grasping, boolean driving){
        switch (stackGrabbingState){
            case SETUP:
                arm.pivotGoToIntake();
                intake.goToStackPosition(intake.getStackPosition());
                arm.centerSteer();
                intake.updateCurrent();
                stackGrabbingWait.reset();
                stackGrabbingState = StackGrabbingState.INTAKING;
                break;

            case INTAKING:
                retract();
                arm.setStopperState(true);
                intake.on(true, 0.75);
                intake.updateCurrent();
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

    public void scoreAsync(double height, boolean bumpUp, double heading){
        switch (scoringState){
            case SETUP:
                arm.centerSteer();
                scoringWait.reset();
                scoringState = ScoringState.EXTENDING;
                break;

            case EXTENDING:
                lift.setHeight(height);
                arm.pivotScore();
                arm.setStopperState(false);
                arm.setBothGrippersState(true);
                // Move on if the lift is all the way up
                if (Utility.withinErrorOfValue(lift.getHeight(), height, 0.5)) {
                    scoringState = ScoringState.WAITING_FOR_ARM_PIVOT;
                }
                break;

            case WAITING_FOR_ARM_PIVOT:
                // Wait til the arm has moved some before steering to avoid crashing the deposit into the bot on the way up
                if (scoringWait.seconds() > 0.25) arm.updateSteer(heading);
                if (scoringWait.seconds() > 1.4) { // Wait for the arm to move all the way
                    arm.setBothGrippersState(false); // Drop the pixels
                    scoringWait.reset();
                    scoringState = ScoringState.WAITING_FOR_PIXELS_DROP;
                }
                break;

            case WAITING_FOR_PIXELS_DROP:
                arm.updateSteer(heading);
                if (scoringWait.seconds() > 0.25){ // Wait for them to fall out
                    scoringWait.reset();
                    scoringState = ScoringState.WAITING_FOR_ARM_RETRACT;
                }
                if (bumpUp && scoringWait.seconds() > 0.07){
                    scoringWait.reset();
                    scoringState = ScoringState.BUMPING_UP;
                }

                break;

            case BUMPING_UP:
                lift.setHeight(height+2);
                arm.updateSteer(heading);
                if (Utility.withinErrorOfValue(lift.getHeight(), height+2, 0.5)) {
                    scoringWait.reset();
                    scoringState = ScoringState.WAITING_FOR_ARM_RETRACT;
                }
                break;

            case WAITING_FOR_ARM_RETRACT:
                arm.pivotGoToIntake();
                arm.centerSteer();
                if (scoringWait.seconds() > 0.5){
                    scoringWait.reset();
                    lift.retract();
                    scoringState = ScoringState.RETRACTING;
                }
                break;

            case RETRACTING:
                lift.retract();
                // Prevent arm hitting stuff near the intake because we spapped it for a speed
                if (Utility.withinErrorOfValue(lift.getHeight(), 0, 2)){
                    arm.pivotGoToIntake();
                } else {
                    arm.preMove();
                }
                // Move on if the lift is all the way down
                if (Utility.withinErrorOfValue(lift.getHeight(), 0, 1)) {
                    scoringState = ScoringState.DONE; // Finish
                }
                break;

            case DONE:
                break;
        }
    }

    public void scoreAsync(double height, boolean bumpUp){
        this.scoreAsync(height, bumpUp, AutoToTele.allianceSide*Math.toRadians(-90));
    }

    public boolean liftIsGoingDown(){
        return scoringState == ScoringState.WAITING_FOR_ARM_RETRACT;
    }
    public void setIntakePos(double pos){
        intake.gotoRawPosition(pos);
    }
    public void intakeOn(){intake.on(false, 0.4);}
    public void intakeOff(){intake.off();}

    // Stuff the ds with info
    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("SCORING MECH");
        telemetry.addData("scoring state", scoringState.name());
        telemetry.addData("scoring wait", scoringWait.milliseconds());
        telemetry.addData("grabbing state", stackGrabbingState.name());
        telemetry.addData("grabbing wait", stackGrabbingWait.milliseconds());
        arm.displayDebug(telemetry);
        lift.disalayDebug(telemetry);
        intake.displayDebug(telemetry);
    }
}
