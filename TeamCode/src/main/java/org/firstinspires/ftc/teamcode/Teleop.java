package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Climber;
import org.firstinspires.ftc.teamcode.hardware.DroneLauncher;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.PurplePixelPusher;
import org.firstinspires.ftc.teamcode.util.DrivingInstructions;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

import java.util.List;

@Config
@TeleOp
public class Teleop extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();
    TeleMecDrive drive;
    PIDFController boardDistanceController;
    public static PIDCoefficients boardDistanceCoeffs = new PIDCoefficients(0.03,0.001,0.001);
    PIDFController boardHeadingController;
    public static PIDCoefficients boardHeadingCoeffs = new PIDCoefficients(0.2,0,0);
    Lift lift;
    Arm arm;
    ElapsedTime pivotTimer = new ElapsedTime();
    ElapsedTime gripperTimer = new ElapsedTime();
    Intake intake;
    Climber climber;
    DroneLauncher launcher;
    PurplePixelPusher ppp;

    public static double liftPosEditStep = 0.6;
    boolean prevLiftInput = false;
    boolean prevHeadingResetInput = false;
    boolean poking = false;
    boolean prevPokingInput = false;

    enum ScoringState {
        INTAKING,
        WAITING_FOR_GRIPPERS,
        PREMOVED,
        SCORING
    }
    ScoringState scoringState = ScoringState.INTAKING;

    // Configuration
    boolean autoRetract = false;
    boolean autoPremove = true;
    boolean boardAssistEnabled = false; // Use the distance sensor and imu to position the bot to the board automatially
    boolean prevBoardAssistInput = false;
    boolean boardAssistActive = false;
    final boolean useBulkreads = true;
    // Telemetry options
    public static boolean debug = true;
    public static boolean instructionsOn = false;

    @Override
    public void runOpMode(){
        // Init
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.4, false);
        boardDistanceController = new PIDFController(boardDistanceCoeffs);
        boardDistanceController.setTargetPosition(3);
        boardHeadingController = new PIDFController(boardHeadingCoeffs);
        boardHeadingController.setTargetPosition(0);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        climber = new Climber(hardwareMap);
        launcher = new DroneLauncher(hardwareMap);
        ppp = new PurplePixelPusher(hardwareMap);
        // Have it up so that if a pixel does get in that area it doesn't break the ppp arm
        ppp.setState(false);

        // Bulk reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        if (useBulkreads) {
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }

        waitForStart();

        matchTimer.reset();
        pivotTimer.reset();
        gripperTimer.reset();

        // START OF TELEOP LOOP
        while (opModeIsActive()){
            // Bulk reads
            if (useBulkreads) {
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }
            }

            // DRIVING
            // Let board assist take control of the sticks if it's enabled and we're probably trying to score on the board
            boardAssistActive = (
                    boardAssistEnabled &&
                    arm.getBoardDistance() < 20 &&
                    scoringState == ScoringState.SCORING
                    //&& Math.abs(drive.getNormalizedHeading()) < 0.17
            );
            if (boardAssistActive){
                drive.driveBoardLocked(
                        gamepad1.left_stick_y,
                        -boardDistanceController.update(arm.getBoardDistance()),
                        gamepad1.right_stick_x,
                        gamepad1.right_trigger
                );
            } else {
                // Drive the bot normally
                drive.driveFieldCentric(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x * 0.8,
                        gamepad1.right_trigger
                );
            }
            // Manually calibrate field centric with a button
            if (gamepad1.share && !prevHeadingResetInput) drive.resetHeading();
            prevHeadingResetInput = gamepad1.share;
            // Enable/disable board assist in case it causes problems
            if (!prevBoardAssistInput && gamepad1.touchpad){
                // Disable for comp, not ready yet
                //boardAssistEnabled = !boardAssistEnabled;
            }
            prevBoardAssistInput = gamepad1.touchpad;

            // ARM AND LIFT CONTROL
            // This method here does most of the heavy work
            updateScoringMech();
            // Edit the extended position with the joystick on gamepad two
            // This works even when the lift is down
            lift.editExtendedPos(-gamepad2.left_stick_y * liftPosEditStep);
            // Update the lift so its pid controller runs, very important
            // But, if you press a special key combo, escape pid control and bring the lift down
            // With raw power to fix potential lift issues
            if (gamepad2.dpad_left && gamepad2.share){
                lift.setRawPowerDangerous(-0.7);
                lift.zero();
            } else if (gamepad2.dpad_right && gamepad2.share) {
                lift.setRawPowerDangerous(1);
                lift.zero();
            }
            else {lift.update();}
            // Update arm
            arm.update();

            // INTAKE CONTROL
            if (gamepad1.b) intake.reverse();
            // Only allow intaking when the arm is there to catch the pixels
            else if ((scoringState == ScoringState.INTAKING || scoringState == ScoringState.WAITING_FOR_GRIPPERS) && lift.getHeight() < 0.5) intake.toggle(gamepad1.a);
            else intake.off();

            // CLIMBER CONTROL
            // Require pressing two keys at once to reduce accidental input
            climber.toggle(gamepad2.left_bumper && gamepad2.right_bumper);
            climber.update();
            // Move the climber arm up if we do anything with it
            if (gamepad2.left_bumper && gamepad2.right_bumper) climber.release();

            // DRONE LAUNCHER CONTROL
            // Require pressing two keys at once to reduce the chance of accidentally shooting it
            if (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8) launcher.release();
            // Could get rid of the else statement but it'll be useful for testing
            // so we don't have to restart the program every time
            else launcher.hold();

            // Send signals to drivers when endgame approaches
            timeUtil.updateAll(matchTimer.milliseconds(), gamepad1, gamepad2);
            // TELEMETRY
            // Show the set height of the lift on a horizontal bar so driver 2 can see it easier than reading a number
            telemetry.addData("Lift target height", lift.getExtendedPos());

            if (debug) {
                telemetry.addData("Board assist enabled", boardAssistEnabled);
                telemetry.addData("Board assist active", boardAssistActive);
                telemetry.addData("Scoring state", scoringState.name());
                telemetry.addData("Board distance target", boardDistanceController.getTargetPosition());
                telemetry.addData("Board distance", arm.getBoardDistance());
                telemetry.addData("Board distance error", boardDistanceController.getLastError());
                drive.displayDebug(telemetry);
                lift.disalayDebug(telemetry);
                intake.displayDebug(telemetry);
                arm.displayDebug(telemetry);
                climber.disalayDebug(telemetry);
                timeUtil.displayDebug(telemetry, matchTimer);
            }
            // Someone should be able to learn how to drive without looking at the source code
            if (instructionsOn) {
              DrivingInstructions.printDrivingInstructions(telemetry);
            }

            telemetry.update();
        } // End of the loop
    }

    boolean hadAnyPixelsWhenPremoved;
    // The big one
    void updateScoringMech(){
        switch (scoringState){
            case INTAKING:
                arm.pivotGoToIntake();
                // Wait to retract the lift until the arm is safely away from the board
                if (pivotTimer.milliseconds() > Arm.pivotAwayFromBordTime) lift.retract();
                // Open the grippers so we can actually intake
                arm.setBothGrippersState(false);
                // Put up the stopper so pixels don't fly out the back
                arm.setStopperState(true);

                // Switch states when bumper pressed
                // Or, (and this'll happen 99% of the time) when it has both pixels
                if ((autoPremove && arm.pixelIsInBottom() && arm.pixelIsInTop()) || (!prevLiftInput && gamepad1.right_bumper)){
                    // Grab 'em and move the arm up
                    arm.setBothGrippersState(true);
                    gripperTimer.reset();
                    scoringState = ScoringState.WAITING_FOR_GRIPPERS;
                }
                break;

            case WAITING_FOR_GRIPPERS:
                if (gripperTimer.milliseconds() > Arm.gripperActuationTime){
                    scoringState = ScoringState.PREMOVED;
                }
                break;

            case PREMOVED:
                arm.preMove();
                // Wait to retract the lift until the arm is safely away from the board
                if (pivotTimer.milliseconds() > Arm.pivotAwayFromBordTime) lift.retract();
                // Just make sure we're still holding on
                arm.setBothGrippersState(true);
                arm.setStopperState(false);
                // Toggle the intake off to prevent sucking in pixels when the arm isn't there
                intake.forceToggleOff();
                // Switch states when bumper pressed
                if (!prevLiftInput && gamepad1.right_bumper){
                    scoringState = ScoringState.SCORING;
                    // Save this info to prevent it from going down right away if you have nothing
                    hadAnyPixelsWhenPremoved = (arm.pixelIsInBottom() || arm.pixelIsInTop());
                }
                break;

            case SCORING:
                arm.pivotScore();
                lift.extend();
                //arm.setStopperState(false);
                // Release the top and bottom individually if we wish
                if (gamepad1.a) {
                    arm.setBottomGripperState(false);
                    poking = false;
                }
                if (gamepad1.y) {
                    arm.setTopGripperState(false);
                    poking = false;
                }
                // But more often used, drop them both at once
                if (gamepad1.left_bumper) {
                    arm.setBothGrippersState(false);
                    poking = false;
                }

                // Toggle the poker
                if (gamepad2.a && !prevPokingInput){
                    poking = !poking;
                }
                prevPokingInput = gamepad2.a;
                arm.setStopperState(poking);

                // Switch states when bumper pressed or both pixels are gone if autoRetract is on
                if (
                        (!prevLiftInput && gamepad1.right_bumper) ||
                        (autoRetract && !(arm.getTopGripperState() || arm.getBottomGripperState()) && !(arm.pixelIsInBottom() || arm.pixelIsInTop()))
                ){
                    scoringState = ScoringState.INTAKING;
                    poking = false;
                    // Reset timer so the clock ticks on the arm being away from the board
                    pivotTimer.reset();
                }
                break;
        }
        prevLiftInput = gamepad1.right_bumper;
    }
}
