package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.ScoringMech;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.vision.workspace.TeamPropDetector;

@Config
@Autonomous
public class Auto1 extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    Camera camera;
    TeamPropDetector propPipeline = new TeamPropDetector();
    ScoringMech scoringMech;

    AutoConstants1 autoConstants;

    int cycleIndex = 0;

    // For the rising egde detectors
    boolean prevCycleIncrease = false;
    boolean prevCycleDecrease = false;
    boolean prevDelayIncrease = false;
    boolean prevDelayDecrease = false;

    // For the giant fsm to run everything asynchronously
    enum AutoState{
        GRABBING_PRELOADS,
        PUSHING_PURPLE,
        SCORING_YELLOW,
        PARKING
    }
    AutoState autoState = AutoState.GRABBING_PRELOADS;

    @Override
    public void runOpMode(){
        // Init
        // Bind stuff to the hardwaremap
        drive = new SampleMecanumDrive(hardwareMap);
        scoringMech = new ScoringMech(hardwareMap);
        camera = new Camera(hardwareMap, propPipeline);
        autoConstants = new AutoConstants1(drive);
        // Juice telemetry speed and allow changing color
        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        ElapsedTime pipelineThrottle = new ElapsedTime();
        ElapsedTime actionTimer = new ElapsedTime();

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

            prevCycleIncrease = gamepad1.dpad_up;
            prevCycleDecrease = gamepad1.dpad_down;
            prevDelayIncrease = gamepad1.dpad_right;
            prevDelayDecrease = gamepad1.dpad_left;

            // Recompute trajectories every second
            if (pipelineThrottle.seconds() > 1){
                // Update stuff
                autoConstants.updateCorrectedSpikeMarkPos(propPipeline.getAnalysis());
                autoConstants.updateTrajectories();

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
        while (opModeIsActive()){
            // One big fsm
            switch (autoState){
                case GRABBING_PRELOADS:
                    // Grab the preload
                    scoringMech.grabJustForPreload();
                    scoringMech.setPPPState(true);
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
                    if (drive.getPoseEstimate().getX() > 50){
                        scoringMech.scoreAsync(5);
                    }
                    if (scoringMech.liftIsGoingDown()){
                        drive.followTrajectorySequenceAsync(autoConstants.park);
                        actionTimer.reset();
                        autoState = AutoState.PARKING;
                    }
                    break;

                case PARKING:
                    // Yay, done!
                    // Once the bot is parked, stop the OpMode
                    if (!drive.isBusy()){
                        stop();
                    }
                    break;
            }
            // Update all the things
            drive.update();
            scoringMech.update();

            // To be used to automatically calibrate field centric
            autoConstants.saveAutoPose();

            // Show telemetry because there are plenty of bugs it should help me fix
            telemetry.addData("auto state", autoState.name());
            telemetry.addData("cycle index", cycleIndex);
            scoringMech.displayDebug(telemetry);
            telemetry.update();
        } // End of while loop
    }
}
