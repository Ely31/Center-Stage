package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm3;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.ExtendoIntake;
import org.firstinspires.ftc.teamcode.hardware.ExtendoScoringMech;
import org.firstinspires.ftc.teamcode.hardware.TapeMeasure;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.TimeUtil;
import org.firstinspires.ftc.teamcode.util.Utility;
import org.firstinspires.ftc.teamcode.vision.workspace.TeamPropDetector2;

import java.util.Objects;

@Config
//@Photon
@Autonomous
public class ExtendoAuto2 extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    Camera camera;
    TeamPropDetector2 propPipeline = new TeamPropDetector2(true);
    ExtendoScoringMech scoringMech;
    TimeUtil timeUtil = new TimeUtil();
    ExtendoAutoConstants2 autoConstants;
    TapeMeasure tapeMeasure;

    // For the rising egde detectors
    boolean prevCycleIncrease = false;
    boolean prevCycleDecrease = false;
    boolean prevDelayIncrease = false;
    boolean prevDelayDecrease = false;
    boolean prevToggleOffset = false;
    boolean prevAvoidYellows = false;

    // For the giant fsm to run everything asynchronously
    enum AutoState{
        GRABBING_PRELOADS,
        PUSHING_PURPLE,
        WAITING_FOR_PPP,
        WAITING_FOR_PPP2,
        SCORING_YELLOW,
        TO_STACK,
        SWEEP_ONE,
        SWEEP_TWO,
        SCORING_WHITE,
        SCORING_WHITE_BACKSTAGE,
        PARKING
    }
    AutoState autoState = AutoState.GRABBING_PRELOADS;

    ElapsedTime pipelineThrottle = new ElapsedTime(1000000000*5); // Start it at 5s so the telemetry pops up right away
    ElapsedTime actionTimer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();

    final double yellowExtendProximity = 23;
    final double whiteExtendProximity = 36;

    @Override
    public void runOpMode(){
        // Init
        // Bind stuff to the hardwaremap
        drive = new SampleMecanumDrive(hardwareMap);
        scoringMech = new ExtendoScoringMech(hardwareMap);
        scoringMech.grabJustForPreload();
        camera = new Camera(hardwareMap, propPipeline);
        autoConstants = new ExtendoAutoConstants2(drive);
        // Juice telemetry speed and allow changing color
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Init loop
        while (!isStarted()&&!isStopRequested()){
            //different auto path choices
            if(gamepad2.cross) autoConstants.setOppositeAuto(true);
            if(gamepad2.triangle) autoConstants.setOppositeAuto(false);

            //different white pixel placement choices
            if (gamepad2.circle) autoConstants.setWhitePixelDropBackstageA(true);
            if (gamepad2.square) autoConstants.setWhitePixelDropBackstageA(false);

            //different backstage drop options
            if(gamepad2.left_trigger > 0.5) autoConstants.setWhitePixelDropBackstageB(true);
            if(gamepad2.right_trigger > 0.5) autoConstants.setWhitePixelDropBackstageB(false);

            // Configure the alliance with the gamepad
            if (gamepad1.circle) autoConstants.setAlliance(1); // Red alliance
            if (gamepad1.cross) autoConstants.setAlliance(-1); // Blue alliance

            // This isn't the best choice of buttons right now
            if (gamepad1.left_bumper){
                autoConstants.setWingSide(true);
                // Do this too for convenience
                autoConstants.setParkingClose(false);
                autoConstants.setNumCycles(1);
            }
            if (gamepad1.right_bumper){
                autoConstants.setWingSide(false);
                // Do this too for convenience
                autoConstants.setParkingClose(true);
                autoConstants.setNumCycles(2);
            }
            // Park options
            if (gamepad1.left_trigger > 0.5) autoConstants.setParkingClose(true);
            if (gamepad1.right_trigger > 0.5) autoConstants.setParkingClose(false);

            if(gamepad2.left_bumper) autoConstants.setTapeMeasurePark(true);
            if(gamepad2.right_bumper) autoConstants.setTapeMeasurePark(false);

            // Buncha rising edge detectors
            if (gamepad1.dpad_up && !prevCycleIncrease) autoConstants.setNumCycles(autoConstants.getNumCycles() + 1);
            if (gamepad1.dpad_down && !prevCycleDecrease) autoConstants.setNumCycles(autoConstants.getNumCycles() - 1);
            if (gamepad1.dpad_right && !prevDelayIncrease) autoConstants.setDelaySeconds(autoConstants.getDelaySeconds() + 1);
            if (gamepad1.dpad_left && !prevDelayDecrease) autoConstants.setDelaySeconds(autoConstants.getDelaySeconds() - 1);
            if (gamepad1.y && !prevToggleOffset) autoConstants.setDropIsOffset(!autoConstants.isDropOffset());
            if (gamepad1.x && !prevAvoidYellows) autoConstants.setAvoidYellows(!autoConstants.isAvoidingYellows());

            prevCycleIncrease = gamepad1.dpad_up;
            prevCycleDecrease = gamepad1.dpad_down;
            prevDelayIncrease = gamepad1.dpad_right;
            prevDelayDecrease = gamepad1.dpad_left;
            prevToggleOffset = gamepad1.y;
            prevAvoidYellows = gamepad1.x;

            // Recompute trajectories every few seconds or every time you make a change
            if (pipelineThrottle.seconds() > 5 || !(Objects.equals(autoConstants.autoConfigToEnglish(), autoConstants.prevConfigToEnglish))){
                // Update stuff
                autoConstants.updateCorrectedSpikeMarkPos(propPipeline.getAnalysis());
                autoConstants.updateTrajectories();
                // switch propPipeline between red and blue based on alliance selected
                propPipeline = new TeamPropDetector2(autoConstants.isRedAlliance());
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
        loopTimer.reset();
        // Save this for tele
        AutoToTele.allianceSide = autoConstants.getAlliance();
        // Used sometimes to avoid potential collisions with a partner
        sleep(Math.abs(autoConstants.getDelaySeconds()*1000));
        while (opModeIsActive()){
            // One big fsm
            switch (autoState){
                case GRABBING_PRELOADS:
                    scoringMech.premove();
                    scoringMech.grabJustForPreload();
                    // Once the claw is shut, premove the v4b, then move on to the next state
                    if (actionTimer.milliseconds() > Arm3.gripperActuationTime){
                        // Set the drive on it's next trajectory
                        moveOnToState(AutoState.PUSHING_PURPLE, autoConstants.dropOffPurplePixel);
                    }
                    break;

                case PUSHING_PURPLE:
                            // Bring intake arm down
                           if(actionTimer.seconds() > 0.8){
                            scoringMech.setIntakePos(0.33);
                            moveOnToState(AutoState.WAITING_FOR_PPP);
                        }
                    break;

                case WAITING_FOR_PPP:
                    if (!drive.isBusy() || Utility.pointsAreWithinDistance(drive.getPoseEstimate(), autoConstants.dropOffPurplePixel.end(), 1)){
                        scoringMech.setIntakePos(0.42);
                        scoringMech.intakeOn();
                        moveOnToState(AutoState.WAITING_FOR_PPP2);
                    }
                    break;

                case WAITING_FOR_PPP2:
                    if (actionTimer.milliseconds() > 50) scoringMech.intakeOff();

                    if (actionTimer.milliseconds() > 200){
                        scoringMech.setIntakePos(ExtendoIntake.verticalPos);
                        moveOnToState(AutoState.SCORING_YELLOW, autoConstants.scoreYellowPixel);
                    }
                    break;

                case SCORING_YELLOW:
                    // If we're close to the board, raise the lift and stuff up
                    // A simple timed delay doesn't work in this case because the length of the path is different depending on drop zone
                    if (Utility.pointsAreWithinDistance(drive.getPoseEstimate(), autoConstants.scoreYellowPixel.end(), (autoConstants.isWingSide() ? yellowExtendProximity - 3 : yellowExtendProximity))){
                        scoringMech.scoreAsync(1.05, true);
                    }
                    if (scoringMech.liftIsGoingDown()){
                        // If not doing cycles, park
                        if (autoConstants.getNumCycles() > 0){
                            autoConstants.updateTrajectories();
                            moveOnToState(AutoState.TO_STACK, autoConstants.toStack);
                        } else {
                            moveOnToState(AutoState.PARKING, autoConstants.park);
                        }
                    }
                    break;

                case TO_STACK:
                    if (actionTimer.seconds() > 2.5){
                        scoringMech.grabOffStackAsync(scoringMech.hasBothPixels(),false);
                    } else scoringMech.scoreAsync(3, true);

                    // Wait to move on until we've started grabbing off the stack because otherwise the sensor vals are out of date
                    if ((scoringMech.hasBothPixels() || (loopTimer.seconds() > 23.5 && scoringMech.hasAPixel())) && actionTimer.seconds() > 4){
                        scoringMech.resetScoringState();
                        addCycle();

                        if(autoConstants.getWhitePixelDropBackstageA() && autoConstants.getNumCycles() == 0){
                            moveOnToState(AutoState.SCORING_WHITE_BACKSTAGE, autoConstants.scoreWhitePixelsBackstageA);
                        }
                        else if(autoConstants.getWhitePixelDropBackstageB() && autoConstants.getNumCycles() == 1){
                            moveOnToState(AutoState.SCORING_WHITE_BACKSTAGE, autoConstants.scoreWhitePixelsBackstageB);
                        }
                        else{
                            moveOnToState(AutoState.SCORING_WHITE, autoConstants.scoreWhitePixels);
                        }
                    }
                    break;

               //Im calling it now, there will be a bug with the interaction between cycle counter and this case
                //TODO: turn this into a state machine for ease of use
                case SCORING_WHITE_BACKSTAGE:
                    if (Utility.pointsAreWithinDistance(drive.getPoseEstimate(), autoConstants.scoreWhitePixelsBackstageA.end(), (autoConstants.isWingSide() ? whiteExtendProximity - 10 : whiteExtendProximity))){
                        scoringMech.scoreAsync(0, false);
                    } else {
                        scoringMech.grabOffStackAsync(true, drive.isBusy());
                    }

                    if (scoringMech.liftIsGoingDown()){
                        autoConstants.updateTrajectories();
                        if (autoConstants.getNumCycles() > 0){
                            scoringMech.resetStackGrabbingState();
                            moveOnToState(AutoState.TO_STACK, autoConstants.toStack);
                        } else {
                            moveOnToState(AutoState.PARKING, autoConstants.park);
                        }
                    }
                    break;

                case SCORING_WHITE:
                    if(autoConstants.getOppositeAuto()){
                        if (Utility.pointsAreWithinDistance(drive.getPoseEstimate(), autoConstants.scoreWhitePixels.end(), (autoConstants.isWingSide() ? whiteExtendProximity - 20 : whiteExtendProximity))){
                            scoringMech.scoreAsync(9, false);
                        }
                        else {
                            scoringMech.grabOffStackAsync(true, drive.isBusy());
                        }
                    }
                    else{
                        if (Utility.pointsAreWithinDistance(drive.getPoseEstimate(), autoConstants.scoreWhitePixels.end(), (autoConstants.isWingSide() ? whiteExtendProximity - 10 : whiteExtendProximity))){
                            scoringMech.scoreAsync(9, false);
                        }
                        else {
                            scoringMech.grabOffStackAsync(true, drive.isBusy());
                        }
                    }

                    if (scoringMech.liftIsGoingDown()){
                        autoConstants.updateTrajectories();
                        if (autoConstants.getNumCycles() > 0){
                            scoringMech.resetStackGrabbingState();
                            moveOnToState(AutoState.TO_STACK, autoConstants.toStack);
                        } else {
                            moveOnToState(AutoState.PARKING, autoConstants.park);
                        }
                    }
                    break;

                case PARKING:
                    // Yay, done!
                    // Keep the scoring mech running so it goes down
                    scoringMech.scoreAsync(3, false);
                    // Once the bot is parked, stop the OpMode
                    if(drive.isBusy() && autoConstants.tapeMeasurePark){
                        tapeMeasure.executeBPM();
                    }
                    else if(!drive.isBusy()){
                        // I swear this method used to be called stop
                        terminateOpModeNow();
                    }
                    break;
            }
            // Update all the things
            drive.update();
            // Only use the sensors we need
            if (scoringMech.getStackGrabbingState() == ExtendoScoringMech.StackGrabbingState.INTAKING){
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
            telemetry.addData("Num cycles", autoConstants.getNumCycles());
            telemetry.addData("Num finished cycles", autoConstants.getNumFinishedCycles());
            telemetry.addData("Time", loopTimer.seconds());
            telemetry.addData("Delay in seconds", autoConstants.getDelaySeconds());
            telemetry.addData("Backstage drop", autoConstants.getWhitePixelDropBackstageA());
            telemetry.addData("Opposite routes", autoConstants.getOppositeAuto());
            telemetry.addData("Tape measure park", autoConstants.isTapeMeasurePark());
            drive.displayDeug(telemetry);
            scoringMech.displayDebug(telemetry);
            timeUtil.displayDebug(telemetry, loopTimer);
            telemetry.update();
        } // End of while loop
    }

    void moveOnToState(AutoState state, TrajectorySequence traj){
        drive.followTrajectorySequenceAsync(traj);
        autoState = state;
        actionTimer.reset();
    }
    void moveOnToState(AutoState state){
        autoState = state;
        actionTimer.reset();
    }
    void addCycle(){
        autoConstants.setNumCycles(autoConstants.getNumCycles()-1);
        autoConstants.addFinishedCycle();
    }
}
