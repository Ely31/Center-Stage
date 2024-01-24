package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.ScoringMech3;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.TimeUtil;
import org.firstinspires.ftc.teamcode.vision.workspace.TeamPropDetector2;

import java.util.Objects;

@Config
//@Photon
@Autonomous
public class CyclesAuto extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    Camera camera;
    TeamPropDetector2 propPipeline = new TeamPropDetector2(true);
    ScoringMech3 scoringMech;
    TimeUtil timeUtil = new TimeUtil();

    CyclesAutoConstants autoConstants;

    // For the rising egde detectors
    boolean prevCycleIncrease = false;
    boolean prevCycleDecrease = false;
    boolean prevDelayIncrease = false;
    boolean prevDelayDecrease = false;
    boolean prevToggleOffset = false;

    // For the giant fsm to run everything asynchronously
    enum AutoState{
        GRABBING_PRELOADS,
        PUSHING_PURPLE,
        SCORING_YELLOW,
        TO_STACK,
        TO_STACKTWO,
        SWEEP_ONE,
        SWEEP_TWO,
        SCORING_WHITE,
        PARKING
    }
    AutoState autoState = AutoState.GRABBING_PRELOADS;

    ElapsedTime pipelineThrottle = new ElapsedTime(1000000000*5); // Start it at 5s so the telemetry pops up right away
    ElapsedTime actionTimer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();

    final double liftExtendXCoord = 25;

    @Override
    public void runOpMode(){
        // Init
        // Bind stuff to the hardwaremap
        drive = new SampleMecanumDrive(hardwareMap);
        scoringMech = new ScoringMech3(hardwareMap);
        scoringMech.grabJustForPreload();
        camera = new Camera(hardwareMap, propPipeline);
        autoConstants = new CyclesAutoConstants(drive);
        // Juice telemetry speed and allow changing color
        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Init loop
        while (!isStarted()&&!isStopRequested()){
            // Configure the alliance with the gamepad
            if (gamepad1.circle) autoConstants.setAlliance(1); // Red alliance
            if (gamepad1.cross) autoConstants.setAlliance(-1); // Blue alliance
            // This isn't the best choice of buttons right now
            if (gamepad1.left_bumper) autoConstants.setWingSide(true);
            if (gamepad1.right_bumper) autoConstants.setWingSide(false);
            // Park options
            if (gamepad1.left_trigger > 0.5) autoConstants.setParkingClose(true);
            if (gamepad1.right_trigger > 0.5) autoConstants.setParkingClose(false);

            // Buncha rising edge detectors
            if (gamepad1.dpad_up && !prevCycleIncrease) autoConstants.setNumCycles(autoConstants.getNumCycles() + 1);
            if (gamepad1.dpad_down && !prevCycleDecrease) autoConstants.setNumCycles(autoConstants.getNumCycles() - 1);
            if (gamepad1.dpad_right && !prevDelayIncrease) autoConstants.setDelaySeconds(autoConstants.getDelaySeconds() + 1);
            if (gamepad1.dpad_left && !prevDelayDecrease) autoConstants.setDelaySeconds(autoConstants.getDelaySeconds() - 1);
            if (gamepad1.y && !prevToggleOffset) autoConstants.setDropIsOffset(!autoConstants.isDropOffset());

            prevCycleIncrease = gamepad1.dpad_up;
            prevCycleDecrease = gamepad1.dpad_down;
            prevDelayIncrease = gamepad1.dpad_right;
            prevDelayDecrease = gamepad1.dpad_left;
            prevToggleOffset = gamepad1.y;

            // Recompute trajectories every few seconds or every time you make a change
            if (pipelineThrottle.seconds() > 5 || !(Objects.equals(autoConstants.autoConfigToEnglish(), autoConstants.prevConfigToEnglish))){
                // Update stuff
                autoConstants.updateCorrectedSpikeMarkPos(propPipeline.getAnalysis());
                autoConstants.updateTrajectories();
                // switch propPipeline between red and blue based on alliance selected
                propPipeline = new TeamPropDetector2(autoConstants.allianceToBool());
                camera.setPipeline(propPipeline);

                drive.setPoseEstimate(autoConstants.startPos);
                // Display auto configuration to telemetry
                autoConstants.addTelemetry(telemetry);
                telemetry.update();
                pipelineThrottle.reset();
            } // End of throttled section
        }

        waitForStart();
        // Stop the camera because we don't need it and it takes computation
        camera.stopStreaming();
        actionTimer.reset();
        // Save this for tele
        AutoToTele.allianceSide = autoConstants.getAlliance();
        // Used sometimes to avoid potential collisions with a partner
        sleep(autoConstants.getDelaySeconds()*1000);
        while (opModeIsActive()){
            // One big fsm
            switch (autoState){
                case GRABBING_PRELOADS:
                    scoringMech.premove();
                    scoringMech.grabJustForPreload();
                    // Once the claw is shut, premove the v4b, then move on to the next state
                    if (actionTimer.milliseconds() > Arm.gripperActuationTime){
                        // Set the drive on it's next trajectory
                        drive.followTrajectorySequenceAsync(autoConstants.dropOffPurplePixel);
                        actionTimer.reset();
                        autoState = AutoState.PUSHING_PURPLE;
                    }
                    break;

                case PUSHING_PURPLE:
                        if (!drive.isBusy()){
                            // Drop the pixel and send it off again
                            scoringMech.setPPPState(false);
                            drive.followTrajectorySequenceAsync(autoConstants.scoreYellowPixel);
                            actionTimer.reset();
                            autoState = AutoState.SCORING_YELLOW;
                        }
                    break;

                case SCORING_YELLOW:
                    // If we're close to the board, raise the lift and stuff up
                    // A simple timed delay doesn't work in this case because the length of the path is different depending on drop zone
                    if (drive.getPoseEstimate().getX() > liftExtendXCoord){
                        scoringMech.scoreAsync(2.75);
                    }
                    if (scoringMech.liftIsGoingDown()){

                        actionTimer.reset();
                        // If not doing cycles, park
                        if (autoConstants.getNumCycles() > 0){
                            drive.followTrajectorySequenceAsync(autoConstants.toStack);
                            scoringMech.resetStackGrabbingState();
                            autoState = AutoState.TO_STACK;
                        } else {
                            drive.followTrajectorySequenceAsync(autoConstants.park);
                            autoState = AutoState.PARKING;
                        }
                        actionTimer.reset();
                    }
                    break;

                case TO_STACK:

                    scoringMech.scoreAsync(3);

                    if (!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(autoConstants.sweepOne);
                        scoringMech.resetStackGrabbingState();
                        autoState = AutoState.SWEEP_ONE;
                        actionTimer.reset();
                    }
                    break;

                case TO_STACKTWO:

                    if (actionTimer.seconds() > 2){
                        scoringMech.grabOffStackAsync(false, false);
                    } else scoringMech.scoreAsync(3);

                    if (!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(autoConstants.scoreWhitePixels);
                        autoState = AutoState.SCORING_WHITE;
                        actionTimer.reset();
                    }
                    break;

                case SWEEP_ONE:
                    scoringMech.grabOffStackAsync(scoringMech.hasBothPixels(), true);
                    if (scoringMech.hasBothPixels()){
                        drive.breakFollowing();
                        autoConstants.setNumCycles(autoConstants.getNumCycles()-1);
                        autoConstants.addFinishedCycle();
                        // Update the trajs because when you break following the start position has to be set to the bot's current position
                        autoConstants.updateTrajectories();
                        drive.followTrajectorySequenceAsync(autoConstants.scoreWhitePixels);
                        autoState = AutoState.SCORING_WHITE;
                        actionTimer.reset();
                        scoringMech.resetScoringState();
                    }
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(autoConstants.sweepTwo);
                        autoState = AutoState.SWEEP_TWO;
                    }
                    break;

                case SWEEP_TWO:
                    scoringMech.grabOffStackAsync(scoringMech.hasBothPixels(), true);
                    if (scoringMech.hasBothPixels()){
                        drive.breakFollowing();
                        autoConstants.setNumCycles(autoConstants.getNumCycles()-1);
                        autoConstants.addFinishedCycle();
                        // Update the trajs because when you break following the start position has to be set to the bot's current position
                        autoConstants.updateTrajectories();
                        drive.followTrajectorySequenceAsync(autoConstants.scoreWhitePixels);
                        autoState = AutoState.SCORING_WHITE;
                        actionTimer.reset();
                        scoringMech.resetScoringState();
                    }
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(autoConstants.sweepOne);
                        autoState = AutoState.SWEEP_ONE;
                    }
                    break;

                case SCORING_WHITE:
                    if (drive.getPoseEstimate().getX() > liftExtendXCoord){
                        scoringMech.scoreAsync(8);
                    } else {
                        scoringMech.grabOffStackAsync(true, true);
                    }

                    if (scoringMech.liftIsGoingDown()){
                        if (autoConstants.getNumCycles() > 0){
                            drive.followTrajectorySequenceAsync(autoConstants.toStackTwo);
                            scoringMech.resetStackGrabbingState();
                            autoState = AutoState.TO_STACKTWO;
                        } else {
                            drive.followTrajectorySequenceAsync(autoConstants.park);
                            autoState = AutoState.PARKING;
                        }
                        actionTimer.reset();
                    }
                    break;

                case PARKING:
                    // Yay, done!
                    // Keep the scoring mech running so it goes down
                    scoringMech.scoreAsync(3);
                    // Once the bot is parked, stop the OpMode
                    if (!drive.isBusy()){
                        // I swear this method used to be called stop
                        terminateOpModeNow();
                    }
                    break;
            }
            // Update all the things
            drive.update();
            // Only use the sensors we need
            // Gosh the formatting on this if statement is ugly
            if (
                    scoringMech.getStackGrabbingState() == ScoringMech3.StackGrabbingState.INTAKING
                    || scoringMech.getStackGrabbingState() == ScoringMech3.StackGrabbingState.KNOCKING
            ){
                scoringMech.update(true, false);
            } else {
                scoringMech.update(false, false);
            }

            // To be used to automatically calibrate field centric
            autoConstants.saveAutoPose();
            // Because I'm curious about loop times
            timeUtil.update(loopTimer.milliseconds());

            // Show telemetry because there are plenty of bugs it should help me fix
            telemetry.addData("Auto state", autoState.name());
            telemetry.addData("Number of cycles", autoConstants.getNumCycles());
            drive.displayDeug(telemetry);
            scoringMech.displayDebug(telemetry);
            timeUtil.displayDebug(telemetry, loopTimer);
            telemetry.update();
        } // End of while loop
    }
}
