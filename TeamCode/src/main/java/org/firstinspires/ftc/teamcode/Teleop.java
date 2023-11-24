package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
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

@Config
@TeleOp
public class Teleop extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();
    TeleMecDrive drive;
    PIDFController boardDistanceController;
    public static PIDCoefficients boardDistanceCoeffs = new PIDCoefficients(0.01,0,0);
    PIDFController boardHeadingController;
    public static PIDCoefficients boardHeadingCoeffs = new PIDCoefficients(0,0,0);
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

    enum ScoringState {
        INTAKING,
        WAITING_FOR_GRIPPERS,
        PREMOVED,
        SCORING
    }
    ScoringState scoringState = ScoringState.INTAKING;

    // Configuration
    boolean autoRetract = true;
    boolean autoPremove = false;
    boolean boardAssistEnabled = false; // Use the distance sensor and imu to position the bot to the board automatially
    boolean prevBoardAssistInput = false;
    boolean boardAssistActive = false;
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
        boardDistanceController.setTargetPosition(5);
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

        waitForStart();
        matchTimer.reset();
        pivotTimer.reset();
        gripperTimer.reset();
        while (opModeIsActive()){
            // Send signals to drivers when endgame approaches
           timeUtil.updateAll(matchTimer.milliseconds(), gamepad1, gamepad2);

            // DRIVING
            // Let board assist take control of the sticks if it's enabled and we're probably facing and close to the board
            boardAssistActive = (
                    boardAssistEnabled &&
                    arm.getBoardDistance() < 30 &&
                    scoringState == ScoringState.SCORING &&
                    Math.abs(drive.getNormalizedHeading()) < 0.17
            );
            if (boardAssistActive){
                drive.driveBoardLocked(
                        gamepad1.left_stick_y,
                        -boardDistanceController.update(arm.getBoardDistance()),
                        gamepad1.right_stick_x * 0.8,
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
                boardAssistEnabled = !boardAssistEnabled;
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
                lift.setRawPowerDangerous(-0.2);
                lift.zero();
            } else
            if (gamepad2.dpad_right && gamepad2.share) {
                lift.setRawPowerDangerous(1);
                lift.zero();
            }
            else lift.update();

            // INTAKE CONTROL
            if (gamepad1.b) intake.reverse();
            // Only allow intaking when the arm is there to catch the pixels
            else if (scoringState == ScoringState.INTAKING || scoringState == ScoringState.WAITING_FOR_GRIPPERS) intake.toggle(gamepad1.a);
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

            // TELEMETRY
            // Show the set height of the lift on a horizontal bar so driver 2 can see it easier than reading a number
            telemetry.addData("Lift target height", lift.getExtendedPos());

            if (debug) {
                telemetry.addData("Board assist enabled", boardAssistEnabled);
                telemetry.addData("Board assist active", boardAssistActive);
                telemetry.addData("Scoring state", scoringState.name());
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
                    // Automatically grab 'em and move the arm up
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
                // Switch states when bumper pressed
                if (!prevLiftInput && gamepad1.right_bumper){
                    scoringState = ScoringState.SCORING;
                }
                break;
            case SCORING:
                arm.pivotScore();
                lift.extend();
                arm.setStopperState(false);
                // Release the top and bottom individually if we wish
                if (gamepad1.square) arm.setBottomGripperState(false);
                if (gamepad1.triangle)arm.setTopGripperState(false);
                // But more often used, drop them both at once
                if (gamepad1.left_bumper) arm.setBothGrippersState(false);

                // Switch states when bumper pressed or both pixels are gone if autoRetract is on
                if ((!prevLiftInput && gamepad1.right_bumper) || (autoRetract && !(arm.pixelIsInBottom() && arm.pixelIsInTop()))){
                    scoringState = ScoringState.INTAKING;
                    // Reset timer so the clock ticks on the arm being away from the board
                    pivotTimer.reset();
                }
                break;
        }
        prevLiftInput = gamepad1.right_bumper;
    }
}
