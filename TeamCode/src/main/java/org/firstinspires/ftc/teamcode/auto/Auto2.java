package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.ScoringMech;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.TimeUtil;
import org.firstinspires.ftc.teamcode.vision.workspace.TeamPropDetector2;

import java.util.Objects;

@Disabled
@Config
//@Photon
@Autonomous
public class Auto2 extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    Camera camera;
    TeamPropDetector2 propPipeline = new TeamPropDetector2(true);
    ScoringMech scoringMech;
    TimeUtil timeUtil = new TimeUtil();

    AutoConstants2 autoConstants;

    int cycleIndex = 0;

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
        PARKING
    }
    AutoState autoState = AutoState.GRABBING_PRELOADS;

    ElapsedTime pipelineThrottle = new ElapsedTime(1000000000*5); // Start it at 5s so the telemetry pops up right away
    ElapsedTime actionTimer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode(){
        // Init
        // Bind stuff to the hardwaremap
        drive = new SampleMecanumDrive(hardwareMap);
        scoringMech = new ScoringMech(hardwareMap);
        scoringMech.grabJustForPreload();
        camera = new Camera(hardwareMap, propPipeline);
        autoConstants = new AutoConstants2(drive);
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
                // This first state is vestigial and isn't needed
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
                    if (drive.getPoseEstimate().getX() > 44){
                        scoringMech.scoreAsync(2.75);
                    }
                    if (scoringMech.liftIsMostlyDown()){
                        drive.followTrajectorySequenceAsync(autoConstants.park);
                        actionTimer.reset();
                        autoState = AutoState.PARKING;
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
            scoringMech.update();

            // To be used to automatically calibrate field centric
            autoConstants.saveAutoPose();
            // Because I'm curious about loop times
            timeUtil.update(loopTimer.milliseconds());

            // Show telemetry because there are plenty of bugs it should help me fix
            telemetry.addData("auto state", autoState.name());
            telemetry.addData("cycle index", cycleIndex);
            drive.displayDeug(telemetry);
            scoringMech.displayDebug(telemetry);
            timeUtil.displayDebug(telemetry, loopTimer);
            telemetry.update();
        } // End of while loop
    }
}
