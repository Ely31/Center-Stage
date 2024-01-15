package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm3;
import org.firstinspires.ftc.teamcode.hardware.DroneLauncher;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.IntegratedClimber;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.DrivingInstructions;
import org.firstinspires.ftc.teamcode.util.TimeUtil;
import org.firstinspires.ftc.teamcode.util.Utility;

import java.util.List;

@Config
//@Photon
@TeleOp
public class Teleop3 extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();
    TeleMecDrive drive;
    Lift lift;
    Arm3 arm;
    ElapsedTime pivotTimer = new ElapsedTime();
    ElapsedTime gripperTimer = new ElapsedTime();
    ElapsedTime doubleTapTimer = new ElapsedTime();
    Intake intake;
    DroneLauncher launcher;
    IntegratedClimber climber;
    ElapsedTime climberTimer = new ElapsedTime();

    PIDFController headingController;
    public static PIDCoefficients headingCoeffs = new PIDCoefficients(0.7,0.005,0.01);

    public static double liftPosEditStep = 0.6;
    boolean prevLiftInput = false;
    boolean prevHeadingResetInput = false;
    boolean poking = false;
    boolean prevPokingInput = false;
    boolean isClimbing = false;
    boolean prevClimbingInput = false;
    boolean usePixelSensors = true;
    boolean prevUsePixelSensorsInput = false;
    final boolean useBulkreads = true;
    public static boolean useHeadingLock = false;
    public static boolean useSlideUpStrategy = true;

    enum ScoringState {
        INTAKING,
        WAITING_FOR_GRIPPERS,
        PREMOVED,
        SCORING,
        SLIDING_UP
    }
    ScoringState scoringState = ScoringState.INTAKING;

    int drivingState;

    // Telemetry options
    public static boolean debug = true;
    public static boolean instructionsOn = false;

    @Override
    public void runOpMode(){
        // Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.3, false);
        lift = new Lift(hardwareMap);
        arm = new Arm3(hardwareMap);
        intake = new Intake(hardwareMap);
        climber = new IntegratedClimber(hardwareMap);
        launcher = new DroneLauncher(hardwareMap);

        headingController = new PIDFController(headingCoeffs);

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
        doubleTapTimer.reset();
        // Automatic feild centric calibration
        drive.setHeadingOffset(AutoToTele.endOfAutoHeading + Math.toRadians(-90*AutoToTele.allianceSide));

        // START OF TELEOP LOOP
        while (opModeIsActive()){
            // Bulk reads
            if (useBulkreads) {
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }
            }

            // DRIVING
            if (!(gamepad1.right_stick_x == 0) || !useHeadingLock) {
                // Drive the bot normally when you give turning input
                drive.driveFieldCentric(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x * 0.8,
                        gamepad1.right_trigger
                );
                // Reset heading controller so it doesn't do weird things when it turns back on
                headingController = new PIDFController(headingCoeffs);
                headingController.setTargetPosition(drive.getHeading());
                drivingState = 0;

            } else {
                // Lock heading with pid controller if you aren't turning
                drive.driveFieldCentric(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        -headingController.update(drive.getHeading()),
                        gamepad1.right_trigger
                );
                drivingState = 1;
            }
            // Manually calibrate field centric with a button
            if (gamepad1.share && !prevHeadingResetInput) {
                drive.resetIMU();
                drive.resetHeadingOffset();
                // Reset heading controller so it doesn't do weird things when it turns back on
                headingController = new PIDFController(headingCoeffs);
                headingController.setTargetPosition(drive.getHeading());
            }
            prevHeadingResetInput = gamepad1.share;

            // Enable/disable autoPremove and autoRetract in case they causes problems
            if (!prevUsePixelSensorsInput && gamepad2.ps){
                usePixelSensors = !usePixelSensors;
            }
            prevUsePixelSensorsInput = gamepad2.ps;

            // ARM AND LIFT CONTROL
            if (!isClimbing) {
                // This method here does most of the heavy work
                updateScoringMech();
                // Edit the extended position with the joystick on gamepad two
                // Only works when the lift is up
                if (scoringState == ScoringState.SCORING) {
                    // If you press the trigger, change the lift height slower
                    if (gamepad2.left_trigger > 0.2) {
                        lift.editExtendedPos(-gamepad2.left_stick_y * liftPosEditStep * 0.5);
                    } else {
                        lift.editExtendedPos(-gamepad2.left_stick_y * liftPosEditStep);
                    }
                }
                // Update the lift so its pid controller runs, very important
                // But, if you press a special key combo, escape pid control and bring the lift down
                // With raw power to fix potential lift issues
                if (gamepad2.dpad_left && gamepad2.share) {
                    lift.setRawPowerDangerous(-0.85);
                    lift.zero();
                } else if (gamepad2.dpad_right && gamepad2.share) {
                    lift.setRawPowerDangerous(1);
                    lift.zero();
                } else {
                    lift.update();
                }
                // Update arm
                // Only poll the sensors we need when we need them to reduce loop times
                if (scoringState == ScoringState.SCORING) arm.update(true, false, true);
                if (scoringState == ScoringState.INTAKING) arm.update(true, true, false);
                if (scoringState == ScoringState.PREMOVED) arm.update(false, false, false);

                // Keep this at 0 until climbing mode is on
                climberTimer.reset();
            } else {
                // CLIMBER CONTROL
                // Climbing mode moves the arm out of the way, escapes all the pid stuff and just runs things with raw power
                arm.pivotGoToIntake();
                // Control lift with left stick
                lift.setRawPowerDangerous(-gamepad2.left_stick_y);
                // Retract the climbing string a bit when you first activate climbing so you don't have much slack to pull in when you hang
                if (climberTimer.seconds() < 0.75){
                    climber.setPower(-1);
                } else {
                    // And control climber with right
                    climber.setPower(-gamepad2.right_stick_y);
                }
            }

            // INTAKE CONTROL
            if (gamepad1.right_stick_button) intake.reverse();
            // Only allow intaking when the arm is there to catch the pixels
            else if (arm.armIsDown()) intake.toggle(gamepad1.left_stick_button);
            else intake.off();

            // DRONE LAUNCHER CONTROL
            // Require pressing two keys at once to reduce the chance of accidentally shooting it
            if (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8) launcher.release();
            // Could get rid of the else statement but it'll be useful for testing
            // so we don't have to restart the program every time
            else launcher.hold();

            // TOGGLE CLIMBING
            if ((gamepad2.left_bumper && gamepad2.right_bumper) && !prevClimbingInput){
                // Reset pid controllers because otherwise the integral part gets extremely high
                if (isClimbing) {
                    lift.setCoefficients(Lift.coeffs);
                }
                isClimbing = !isClimbing;
            }
            prevClimbingInput = gamepad2.left_bumper && gamepad2.right_bumper;

            // TELEMETRY
            if (debug) {
                telemetry.addData("Using pixel sensors", usePixelSensors);
                telemetry.addData("Climbing", isClimbing);
                telemetry.addData("Driving mode", drivingState);
                telemetry.addData("Target heading", headingController.getTargetPosition());
                telemetry.addData("Heading", drive.getHeading());
                telemetry.addData("Heading error", headingController.getLastError());
                telemetry.addData("Scoring state", scoringState.name());
                telemetry.addLine();
                telemetry.addLine("SUBSYSTEMS");
                telemetry.addLine();
                drive.displayDebug(telemetry);
                lift.disalayDebug(telemetry);
                intake.displayDebug(telemetry);
                arm.displayDebug(telemetry);
                climber.disalayDebug(telemetry);
                timeUtil.update(matchTimer.milliseconds());
                timeUtil.displayDebug(telemetry, matchTimer);
            }
            // Someone should be able to learn how to drive without looking at the source code
            if (instructionsOn) {
              DrivingInstructions.printDrivingInstructions(telemetry);
            }
            telemetry.update();
        } // End of the loop

        if (isStopRequested()){
            // Open both grippers if you stop the program, makes it less work for us extracting pixels
            arm.setBothGrippersState(false);
        }
    }

    boolean hadAnyPixelsWhenPremoved;
    // The big one
    void updateScoringMech(){
        switch (scoringState){
            case INTAKING:
                arm.pivotGoToIntake();
                // Wait to retract the lift until the arm is safely away from the board
                if (pivotTimer.milliseconds() > Arm3.pivotAwayFromBordTime) {
                    lift.retract();
                }
                // Put up the stopper so pixels don't fly out the back, but only after the arm's back to avoid hooking a pixel on it
                if (pivotTimer.milliseconds() > 500) {
                    arm.setStopperState(true);
                }
                // If we had pixels when premoved and now moved back down to intaking,
                // hold onto them until the arm gets all the way down so they don't fly out.
                // Open them to intake once the arm gets all the way there.
                arm.setBothGrippersState(!arm.armIsDown());
                // Reset poker
                poking = false;
                // Switch states when bumper pressed
                // Or, (and this'll happen 95% of the time) when it has both pixels
                if ((usePixelSensors && arm.pixelIsInBottom() && arm.pixelIsInTop()) || (!prevLiftInput && gamepad2.right_bumper)){
                    // Grab 'em and move the arm up
                    arm.setBothGrippersState(true);
                    gripperTimer.reset();
                    scoringState = ScoringState.WAITING_FOR_GRIPPERS;
                }
                break;

            case WAITING_FOR_GRIPPERS:
                if (gripperTimer.milliseconds() > Arm3.gripperActuationTime){
                    scoringState = ScoringState.PREMOVED;
                    doubleTapTimer.reset();
                }
                break;

            case PREMOVED:
                arm.preMove();
                // Wait to retract the lift until the arm is safely away from the board
                if (pivotTimer.milliseconds() > Arm3.pivotAwayFromBordTime) lift.retract();
                // Just make sure we're still holding on
                arm.setBothGrippersState(true);
                arm.setStopperState(false);
                // Toggle the intake off to prevent sucking in pixels when the arm isn't there
                intake.forceToggleOff();
                // Reset poker
                poking = false;

                // Switch states when bumper pressed
                // Don't go to scoring if the arm has just been premoved though, this happens when the automatic raises it but the driver
                // tries to manually raise it at the same time. This timer prevents that.
                if (!prevLiftInput && gamepad2.right_bumper && doubleTapTimer.milliseconds() > 450){
                    scoringState = ScoringState.SCORING;
                    // Save this info to prevent it from going down right away if you have nothing
                    hadAnyPixelsWhenPremoved = (arm.pixelIsInBottom() || arm.pixelIsInTop());
                }
                // Go back to intaking if the arm pulled up before getting both pixels
                if (gamepad2.dpad_left){
                    scoringState = ScoringState.INTAKING;
                }
                break;

            case SCORING:
                arm.pivotScore();
                lift.extend();
                // Release the top and bottom individually if we wish
                if (gamepad2.a) {
                    arm.setBottomGripperState(false);
                    poking = false;
                }
                if (gamepad2.y) {
                    arm.setTopGripperState(false);
                    poking = false;
                }
                // But more often used, drop them both at once
                if (gamepad2.left_bumper) {
                    arm.setBothGrippersState(false);
                    poking = false;
                }

                // Toggle the poker
                if (gamepad2.b && !prevPokingInput){
                    poking = !poking;
                }
                prevPokingInput = gamepad2.b;
                arm.setStopperState(poking);

                // Switch states when the bumper is pressed or both pixels are gone if autoRetract is on
                if (
                        (!prevLiftInput && gamepad2.right_bumper) ||
                        (usePixelSensors && !(arm.getTopGripperState() || arm.getBottomGripperState()) && !(arm.pixelIsInBottom() || arm.pixelIsInTop()))
                ){
                    if (useSlideUpStrategy) {
                        scoringState = ScoringState.SLIDING_UP;
                        lift.setExtendedPos(lift.getExtendedPos() + 2);
                    }
                    else scoringState = ScoringState.INTAKING;
                    // Reset timer so the clock ticks on the arm being away from the board
                    pivotTimer.reset();
                }
                // Go back to premoved if we wish
                if (gamepad2.dpad_left){
                    scoringState = ScoringState.PREMOVED;
                    pivotTimer.reset();
                }
                break;

            case SLIDING_UP:
                lift.extend();
                // Once it's gone up enough, switch states and retract
                if (Utility.withinErrorOfValue(lift.getHeight(), lift.getExtendedPos(), 0.5)) {
                    // Reset that back to normal because we temporarily changed it
                    lift.setExtendedPos(lift.getExtendedPos() - 2);
                    scoringState = ScoringState.INTAKING;
                    // Reset timer so the clock ticks on the arm being away from the board
                    pivotTimer.reset();
                }
        }
        prevLiftInput = gamepad2.right_bumper;
    }
}
