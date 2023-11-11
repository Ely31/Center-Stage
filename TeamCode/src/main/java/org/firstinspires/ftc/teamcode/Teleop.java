package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Climber;
import org.firstinspires.ftc.teamcode.hardware.DroneLauncher;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.util.DrivingInstructions;
import org.firstinspires.ftc.teamcode.util.TimeUtil;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
@TeleOp
public class Teleop extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();
    TeleMecDrive drive;
    Lift lift;
    Arm arm;
    Intake intake;
    Climber climber;
    DroneLauncher launcher;
    double drivingSpeedMultiplier = 1;
    // Lift constants
    public static double liftPosEditStep = 0.2;
    public static double liftRawPowerAmount = 0.2;

    boolean prevLiftInput = false;

    enum ScoringState {
        INTAKING,
        PREMOVED,
        SCORING
    }

    ScoringState scoringState = ScoringState.INTAKING;

    // Telemetry options
    public static boolean debug = true;
    public static boolean instructionsOn = false;

    @Override
    public void runOpMode(){
        // Init
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.4, false);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        climber = new Climber(hardwareMap);
        launcher = new DroneLauncher(hardwareMap);

        waitForStart();
        matchTimer.reset();
        while (opModeIsActive()){
            // Send signals to drivers when endgame approaches
           timeUtil.updateAll(matchTimer.milliseconds(), gamepad1, gamepad2);

            // Slow down the bot when scoring
            //if (scoringState == ScoringState.SCORING) drivingSpeedMultiplier = 0.3;
            //else drivingSpeedMultiplier = 1;
            // Drive the bot
            drive.driveFieldCentric(
                    gamepad1.left_stick_x * drivingSpeedMultiplier,
                    gamepad1.left_stick_y * drivingSpeedMultiplier,
                    gamepad1.right_stick_x * drivingSpeedMultiplier * 0.8,
                    gamepad1.right_trigger);

            // Manually calibrate field centric with a button
            if (gamepad1.share) drive.resetHeading();

            // ARM AND LIFT CONTROL
            // This method here does most of the work
            updateScoringMech();
            // Edit the extended position with the dpad on gamepad two
            // This works even when the lift is down
            if (gamepad2.dpad_up)   lift.editExtendedPos(liftPosEditStep);
            if (gamepad2.dpad_down) lift.editExtendedPos(-liftPosEditStep);
            // Update the lift so its pid controller runs, very important
            // But, if you press a special key combo, escape pid control and bring the lift down
            // With raw power to fix potential lift issues
            if (gamepad2.dpad_left && gamepad2.share){
                lift.setRawPowerDangerous(-liftRawPowerAmount);
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
            else if (scoringState == ScoringState.INTAKING) intake.toggle(gamepad1.a);
            else intake.off();

            // CLIMBER CONTROL
            // Require pressing two keys at once to reduce accidental input
            climber.toggle(gamepad2.left_bumper && gamepad2.right_bumper);
            climber.update();
            // Move the arm up if we do anything with it
            if (gamepad2.left_bumper && gamepad2.right_bumper) climber.release();

            // DRONE LAUNCHER CONTROL
            // Require pressing two keys at once to reduce the chance of accidentally shooting it
            if (gamepad2.left_bumper && gamepad2.right_trigger > 0.1) launcher.release();
            else launcher.hold();

            // TELEMETRY
            // Show the set height of the lift on a horizontal bar so driver 2 can see it easier than reading a number
            telemetry.addData("Lift target height", lift.getExtendedPos());
            telemetry.addLine(Utility.generateTelemetryTrackbar(Lift.minHeight, Lift.maxHeight, lift.getExtendedPos(),10));
            // Heck, stack two on top of each other so it's even bigger
            telemetry.addLine(Utility.generateTelemetryTrackbar(Lift.minHeight, Lift.maxHeight, lift.getExtendedPos(),10));

            if (debug) {
                telemetry.addData("heading", drive.getHeading());
                lift.disalayDebug(telemetry);
                intake.displayDebug(telemetry);
                arm.displayDebug(telemetry);
                climber.disalayDebug(telemetry);
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

    void updateScoringMech(){
        switch (scoringState){
            case INTAKING:
                arm.pivotGoToIntake();
                // Open the grippers so we can actually intake
                arm.setBothGrippersState(false);
                lift.retract();
                // Switch states when bumper pressed
                if (!prevLiftInput && gamepad1.left_bumper){
                    scoringState = ScoringState.SCORING;
                }
                // Or, (and this'll happen 99% of the time) when it has both pixels
                if (arm.pixelIsInBottom() && arm.pixelIsInTop()){
                    // Automatically grab 'em and move the arm up
                    arm.setBothGrippersState(true);
                    scoringState = ScoringState.PREMOVED;
                }
                break;
            case PREMOVED:
                arm.preMove();
                // Just make sure we're still holding on
                arm.setBothGrippersState(true);
                lift.retract();
                // Switch states when bumper pressed
                if (!prevLiftInput && gamepad1.left_bumper){
                    scoringState = ScoringState.SCORING;
                }
                break;
            case SCORING:
                arm.pivotScore();
                lift.extend();
                // Release the top and bottom individually if we wish
                if (gamepad1.square) arm.setBottomGripperState(false);
                if (gamepad1.triangle)arm.setTopGripperState(false);
                // But more often used, drop them both at once
                if (gamepad1.right_bumper) arm.setBothGrippersState(false);
                // TODO: Implement autoretract after both pixels are dropped
                // Switch states when bumper pressed
                if (!prevLiftInput && gamepad1.left_bumper){
                    scoringState = ScoringState.INTAKING;
                }
                break;
        }
        prevLiftInput = gamepad1.left_bumper;
    }
}
