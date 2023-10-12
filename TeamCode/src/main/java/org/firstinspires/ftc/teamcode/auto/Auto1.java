package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.AutoScoringMechOld;
import org.firstinspires.ftc.teamcode.hardware.DualMotorLift;
import org.firstinspires.ftc.teamcode.hardware.PivotingCamera;
import org.firstinspires.ftc.teamcode.vision.old.SignalPipeline;

//this autonomous is meant for if you start on the left side of the field
//regular is the red side of the field, -1 is blue side of the field
@Config
@Autonomous
public class Auto1 extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    PivotingCamera camera;
    SignalPipeline signalPipeline = new SignalPipeline();
    AutoScoringMechOld scoringMech;

    AutoConstants1 autoConstants;

    int cycleIndex = 0;

    // For the rising egde detectors
    boolean prevCycleIncrease = false;
    boolean prevCycleDecrease = false;

    // For the giant fsm to run everything asynchronously
    enum AutoState{
        GRABBING_PRELOAD,
        SCORING_PRELOAD,
        WAITING_FOR_CONE_GRAB,
        TO_JUNCTION,
        WAITING_TIL_CLEAR_OF_JUNCTION,
        TO_STACK,
        PARKING
    }
    AutoState autoState = AutoState.GRABBING_PRELOAD;

    @Override
    public void runOpMode(){
        // Init
        // Bind stuff to the hardwaremap
        drive = new SampleMecanumDrive(hardwareMap);
        scoringMech = new AutoScoringMechOld(hardwareMap);
        camera = new PivotingCamera(hardwareMap, signalPipeline);
        autoConstants = new AutoConstants1(drive);
        // Juice telemetry speed
        telemetry.setMsTransmissionInterval(100);

        ElapsedTime pipelineThrottle = new ElapsedTime();
        ElapsedTime actionTimer = new ElapsedTime();

        // Init loop
        while (!isStarted()&&!isStopRequested()){
            // Configure the alliance with the gamepad
            if (gamepad1.circle) autoConstants.setSide(1); // Red alliance
            if (gamepad1.cross) autoConstants.setSide(-1); // Blue alliance

            // Recompute trajectories every second
            if (pipelineThrottle.seconds() > 1){
                // Update stuff
                autoConstants.updateDropLocationFromVisionResult(signalPipeline.getParkPos());
                autoConstants.updateParkPos(autoConstants.getDropLocation());
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

        while (opModeIsActive()){
            // One big fsm
            switch (autoState){
                case GRABBING_PRELOAD:
                    // Grab the preload
                    scoringMech.closeClaw();
                    // Once the claw is shut, premove the v4b, then move on to the next state
                    if (actionTimer.milliseconds() > Arm.clawActuationTime){
                        //scoringMech.preMoveV4b();
                        // Set the drive on it's next trajectory
                        drive.followTrajectorySequenceAsync(autoConstants.dropOffPurplePixel);
                        actionTimer.reset();
                        autoState = AutoState.SCORING_PRELOAD;
                    }
                    break;

                case SCORING_PRELOAD:
                        if (actionTimer.seconds() > 2){
                            scoringMech.scoreWithBracer(DualMotorLift.mediumPos);
                        }
                        if (scoringMech.liftIsMostlyDown()){
                            // Send it off again
                            drive.followTrajectorySequenceAsync(autoConstants.park);
                            actionTimer.reset();
                            // Reset the scoring fsm so it can run again next time
                            scoringMech.resetScoringState();
                            autoState = AutoState.PARKING;
                        }
                    break;

                case PARKING:
                    // Yay, done
                    // Once the bot is parked, stop the OpMode
                    if (!drive.isBusy()){
                        stop();
                    }
                    break;
            }
            // Update all the things
            drive.update();
            scoringMech.updateLift();

            // To be used to automatically calibrate field centric
            autoConstants.saveAutoPose();

            // Show telemetry because there are plenty of bugs it should help me fix
            telemetry.addData("auto state", autoState.name());
            telemetry.addData("cycle index", cycleIndex);
            telemetry.addData("number of cycles", autoConstants.getNumCycles());
            scoringMech.displayAutoMechDebug(telemetry);
            telemetry.update();
        }
    }
}
