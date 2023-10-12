package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.ScoringMechOld;
import org.firstinspires.ftc.teamcode.util.DrivingInstructions;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@Config
@Disabled
@TeleOp
public class OldTeleop extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();
    TeleMecDrive drive;
    double drivingSpeedMultiplier;
    ScoringMechOld scoringMechOld;

    ElapsedTime clawActuationTimer = new ElapsedTime();
    ElapsedTime pivotActuationTimer = new ElapsedTime();

    // Claw fsm enum
    enum GrabbingState {
        OPEN,
        ClOSED,
        WAITING_OPEN
    }
    GrabbingState grabbingState = GrabbingState.OPEN;

    enum ScoringState{
        RETRACTED,
        PREMOVED,
        SCORING
    }
    ScoringState scoringState = ScoringState.RETRACTED;

    public static boolean autoRetract = true;

    // Stuff for rising edge detectors
    boolean prevClawInput = false;
    boolean prevScoringInput = false;

    boolean prevStackIndexUpInput = false;
    boolean prevStackIndexDownInput = false;

    boolean hackyExtendSignal = false;

    // Lift constants
    public static double liftPosEditStep = 0.15;
    public static double liftRawPowerAmount = 0.2;

    // Telemetry options
    public static boolean debug = true;
    public static boolean instructionsOn = true;

    @Override
    public void runOpMode(){
        // Init
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.4, false);
        scoringMechOld = new ScoringMechOld(hardwareMap);

        waitForStart();
        matchTimer.reset();
        clawActuationTimer.reset();
        pivotActuationTimer.reset();
        while (opModeIsActive()){
            // Send signals to drivers when endgame approaches
           timeUtil.updateAll(matchTimer.milliseconds(), gamepad1, gamepad2);

            // Slow down the bot when scoring
            if (scoringState == ScoringState.SCORING) drivingSpeedMultiplier = 0.3;
            else drivingSpeedMultiplier = 1;
            // Drive the bot
            drive.driveFieldCentric(
                    gamepad1.left_stick_x * drivingSpeedMultiplier,
                    gamepad1.left_stick_y * drivingSpeedMultiplier,
                    gamepad1.right_stick_x * drivingSpeedMultiplier * 0.8,
                    gamepad1.right_trigger);

            // Manually calibrate field centric with a button
            if (gamepad1.share) drive.resetHeading();

            // ARM AND LIFT CONTROL
            // Edit things
            // Switch active junction using the four buttons on gamepad one
            if (gamepad1.cross)     scoringMechOld.setActiveScoringJunction(0);
            if (gamepad1.square)    scoringMechOld.setActiveScoringJunction(1);
            if (gamepad1.triangle)  scoringMechOld.setActiveScoringJunction(2);
            if (gamepad1.circle)    scoringMechOld.setActiveScoringJunction(3);

            // This bit is hacked in to change the behavior without ripping up too much code
            // If you press any of the buttons, make the signal true. If not, it's false.
            hackyExtendSignal = gamepad1.cross || gamepad1.square || gamepad1.triangle || gamepad1.circle;

            // Edit the current level with the dpad on gamepad two
            if (gamepad2.dpad_up)   scoringMechOld.editCurrentLiftPos(liftPosEditStep);
            if (gamepad2.dpad_down) scoringMechOld.editCurrentLiftPos(-liftPosEditStep);
            // Edit retracted pose for grabbing off the stack using rising edge detectors
            if (gamepad2.triangle && !prevStackIndexUpInput) {
                scoringMechOld.setStackIndex(scoringMechOld.getStackIndex()+1);
                scoringMechOld.setRetractedGrabbingPose(scoringMechOld.getStackIndex());
            }
            prevStackIndexUpInput = gamepad2.triangle;
            if (gamepad2.cross && !prevStackIndexDownInput) {
                scoringMechOld.setStackIndex(scoringMechOld.getStackIndex()-1);
                scoringMechOld.setRetractedGrabbingPose(scoringMechOld.getStackIndex());
            }
            prevStackIndexDownInput = gamepad2.cross;

            // Claw control
            updateClaw(gamepad1.left_bumper);

            // Scoring mech (lift and v4b) control
            updateScoringmech();

            // Update the lift so its pid controller runs, very important
            // But, if you press a special key combo, escape pid control and bring the lift down
            // With raw power to fix a lift issue
            if (gamepad2.dpad_left && gamepad2.share){
                scoringMechOld.setRawLiftPowerDangerous(-liftRawPowerAmount);
                scoringMechOld.zeroLift();
            } else
            if (gamepad2.dpad_right && gamepad2.share) {
                scoringMechOld.setRawLiftPowerDangerous(1);
                scoringMechOld.zeroLift();
            }
            else scoringMechOld.updateLift();


            // TELEMETRY
            if (debug) {
                telemetry.addData("scoring state", scoringState.name());
                telemetry.addData("active junction", scoringMechOld.getActiveScoringJunction());
                telemetry.addData("grabbing state", grabbingState.name());
                telemetry.addData("stack index", scoringMechOld.getStackIndex());
                telemetry.addData("heading", drive.getHeading());
                scoringMechOld.displayDebug(telemetry);
                telemetry.addData("avg loop time (ms)", timeUtil.getAverageLoopTime());
                telemetry.addData("period", timeUtil.getPeriod());
                telemetry.addData("time", matchTimer.seconds());
            }
            // Someone should be able to learn how to drive without looking at the source code
            if (instructionsOn) {
              DrivingInstructions.printDrivingInstructions(telemetry);
            }

            telemetry.update();
        } // End of the loop
    }

    // Methods for handling the complex interactions of the scoring mech and claw
    void updateClaw(boolean input){
        switch(grabbingState){
            case OPEN:
                scoringMechOld.openClaw();
                // If the button is pressed for the first time or the sensor detects a cone, close the claw
                if ((input && !prevClawInput) || scoringMechOld.getConeStatus()){
                    grabbingState = GrabbingState.ClOSED;
                }
                break;
            case ClOSED:
                scoringMechOld.closeClaw();
                if (input && !prevClawInput){
                    grabbingState = GrabbingState.WAITING_OPEN;
                }
                break;
            case WAITING_OPEN:
                scoringMechOld.openClaw();
                // If you press the button again, close it
                if (input && !prevClawInput){
                    grabbingState = GrabbingState.ClOSED;
                }
                // If it stops seeing the cone, go back to the open state, where it starts to look for one again
                if (!scoringMechOld.getConeStatus()){
                    grabbingState = GrabbingState.OPEN;
                }
                break;
        }
        prevClawInput = input;
        // Keep the timer at zero until we want it to start ticking, when the claw is closed
        if (!(grabbingState == GrabbingState.ClOSED)) clawActuationTimer.reset();
    }

    void updateScoringmech(){
        // Run the scoring fsm
        switch (scoringState){
            case RETRACTED:
                scoringMechOld.retract(pivotActuationTimer.milliseconds());
                // Handle bracer stuff
                if (pivotActuationTimer.milliseconds() > Arm.pivotActuationTime + 500){
                    // The servo doesn't need to be working once everything is back inside the bot
                    scoringMechOld.extendBracer();
                }
                // Don't retract the bracer when doing ground junctions, it would hit the bot
                else if(!(scoringMechOld.getActiveScoringJunction() == 0)){
                    scoringMechOld.retractBracer();
                }

                if (grabbingState == GrabbingState.ClOSED && clawActuationTimer.milliseconds() > Arm.clawActuationTime){
                    scoringState = ScoringState.PREMOVED;
                }
                break;

            case PREMOVED:
                scoringMechOld.retractPremoved(pivotActuationTimer.milliseconds());

                if (pivotActuationTimer.milliseconds() > Arm.pivotActuationTime + 300){
                    // The servo doesn't need to be working once everything is back inside the bot
                    scoringMechOld.extendBracer();
                }
                // Don't retract the bracer when doing ground junctions, it would hit the bot
                else if(!(scoringMechOld.getActiveScoringJunction() == 0)){
                    scoringMechOld.retractBracer();
                }

                if (((gamepad1.left_trigger > 0) && !prevScoringInput) || hackyExtendSignal) {
                    scoringState = ScoringState.SCORING;
                }
                if (grabbingState == GrabbingState.OPEN || grabbingState == GrabbingState.WAITING_OPEN){
                    scoringState = ScoringState.RETRACTED;
                }
                break;

            case SCORING:
                scoringMechOld.score();
                scoringMechOld.extendBracer();
                // Retract if you press the retract button
                if ((gamepad1.left_trigger > 0) && !prevScoringInput) {
                    pivotActuationTimer.reset();
                    scoringState = ScoringState.RETRACTED;
                }

                if (autoRetract) {
                    // Or, retract automatically when you drop the cone
                    if (grabbingState == GrabbingState.OPEN) {
                        pivotActuationTimer.reset();
                        scoringState = ScoringState.RETRACTED;
                    }
                }
                break;
        }
        prevScoringInput = (gamepad1.left_trigger > 0);
    }
}