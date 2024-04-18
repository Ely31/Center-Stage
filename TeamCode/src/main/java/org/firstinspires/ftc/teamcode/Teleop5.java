package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Climber;
import org.firstinspires.ftc.teamcode.hardware.DroneLauncher;
import org.firstinspires.ftc.teamcode.hardware.ExtendoIntakeAngleHolding;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.SteeringArm;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.TimeUtil;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
//@Photon
@TeleOp
public class Teleop5 extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();
    TeleMecDrive drive;
    Lift lift;
    SteeringArm arm;
    ElapsedTime pivotTimer = new ElapsedTime();
    ElapsedTime gripperTimer = new ElapsedTime();
    ElapsedTime doubleTapTimer = new ElapsedTime();
    ExtendoIntakeAngleHolding intake;
    DroneLauncher launcher;
    Climber climber;
    ElapsedTime climberTimer = new ElapsedTime();

    PIDFController headingController;
    public static PIDCoefficients headingCoeffs = new PIDCoefficients(2, 0.005, 0.2); // old vals (0.7,0.005,0.01)

    public static double liftPosEditStep = 0.45;
    boolean prevLiftInput = false;
    boolean prevCalibratingLift = false;
    boolean calibratingLift = false;
    boolean prevHeadingResetInput = false;
    boolean poking = false;
    boolean prevPokingInput = false;
    boolean isClimbing = false;
    boolean prevClimbingInput = false;
    boolean usePixelSensors = true;
    boolean prevUsePixelSensorsInput = false;
    boolean prevStackUp = false;
    boolean prevStackDown = false;
    boolean dontRetractThisTime = false;
    public static boolean displayDebugTelemetry = true;

    enum ScoringState {
        INTAKING,
        WAITING_FOR_GRIPPERS,
        PREMOVED,
        SCORING,
        SLIDING_UP,
        BUMPING_UP
    }

    enum ClimbingState{
        REDUCE_SLACK,
        HOLD,
        CLIMB
    }

    ScoringState scoringState = ScoringState.INTAKING;
    ClimbingState climbingState = ClimbingState.REDUCE_SLACK;

    int drivingState;
    ElapsedTime drivingTimer = new ElapsedTime();
    boolean hasCalibratedFC = false;
    double climberSlackPullTime = 1.3;

    @Override
    public void runOpMode(){
        // Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.3, false);
        lift = new Lift(hardwareMap);
        arm = new SteeringArm(hardwareMap);
        intake = new ExtendoIntakeAngleHolding(hardwareMap);
        climber = new Climber(hardwareMap);
        launcher = new DroneLauncher(hardwareMap);

        headingController = new PIDFController(headingCoeffs);

        waitForStart();

        matchTimer.reset();
        pivotTimer.reset();
        gripperTimer.reset();
        doubleTapTimer.reset();
        // Automatic feild centric calibration
        drive.setHeadingOffset(AutoToTele.endOfAutoHeading + Math.toRadians(-90*AutoToTele.allianceSide));

        // START OF TELEOP LOOP
        while (opModeIsActive()){
            // DRIVING

            if (false && gamepad1.right_stick_x == 0 && drivingTimer.seconds() > 0.3){
                // Lock heading with pid controller if you aren't turning
                drive.driveFieldCentric(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        -headingController.update(drive.getHeading()),
                        gamepad1.right_trigger
                    );
                    drivingState = 2;
            } else {
                drivingState = 0;
                // Drive the bot normally
                drive.driveFieldCentric(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x * 0.8,
                        gamepad1.right_trigger
                );
                // Reset pid controllers so they don't do weird things when they turn back on
                resetHeadingController();
            }
            if (gamepad1.right_stick_x != 0){
                drivingTimer.reset();
            }

            // Manually calibrate field centric with a button
            if (gamepad1.share && !prevHeadingResetInput) {
                drive.resetIMU();
                drive.resetHeadingOffset();
                resetHeadingController();
                hasCalibratedFC = true;
            }
            prevHeadingResetInput = gamepad1.share;

            // Enable/disable autoPremove and autoRetract in case they causes problems
            if (!prevUsePixelSensorsInput && gamepad2.ps){
                usePixelSensors = !usePixelSensors;
            }
            prevUsePixelSensorsInput = gamepad2.ps;

            /* Enable/disable using the board distance sensor
            if (!prevUseBoardSensorInput && gamepad1.ps){
                useBoardSensor = !useBoardSensor;
            }
            prevUseBoardSensorInput = gamepad1.ps;
             */

            // ARM AND LIFT CONTROL
            if (!isClimbing) {
                // Edit the extended position with the joystick on gamepad two
                // Only works when the lift is up
                if (scoringState == ScoringState.SCORING) {
                    // If you press the trigger, change the lift height slower
                    if (gamepad2.right_trigger > 0.2) {
                        lift.editExtendedPos(-gamepad2.left_stick_y * liftPosEditStep * 0.5);
                    } else {
                        lift.editExtendedPos(-gamepad2.left_stick_y * liftPosEditStep);
                    }
                }
                // Update the entire scoring mech, very important
                // But, if you press a special key combo, escape pid control and all the state machine stuff to bring the lift down
                // With raw power to fix potential lift issues
                calibratingLift = ((gamepad2.dpad_left || gamepad2.dpad_right) && gamepad2.share);
                if (calibratingLift) {
                    // Check which button it was and apply power in that direction
                    if (gamepad2.dpad_right) {lift.setRawPowerDangerous(0.5);}
                    else {lift.setRawPowerDangerous(-0.35);}
                    // Turn off all the other stuff on the lift temporarily
                    resetLiftController();
                } else {
                    // Zero the lift just once after you move it with raw power because if you continuously zero it then it gets zero power
                    // When you try to move it with raw power
                    if (prevCalibratingLift) lift.zero();
                    // This method here does all of the heavy work
                    updateScoringMech();
                }
                prevCalibratingLift = calibratingLift;
            } else {
                updateClimibingSystem();
            }

            // INTAKE CONTROL
            if (gamepad1.right_stick_button) intake.reverse();
            // Only allow intaking when the arm is there to catch the pixels
            else if (arm.armIsDown() && (scoringState == ScoringState.INTAKING || scoringState == ScoringState.WAITING_FOR_GRIPPERS)){
                intake.toggle(gamepad1.left_stick_button, false, 0.75);
                // Move the intake back down after it was up if we turn it on
                if (intake.getToggledStatus()) intake.goToStackPosition(intake.getStackPosition());

                // Edit intake stack position
                if (gamepad2.dpad_up && ! prevStackUp){
                    intake.goToStackPosition(intake.getStackPosition() + 1);
                }
                if (gamepad2.dpad_down && ! prevStackDown){
                    intake.goToStackPosition(intake.getStackPosition() - 1);
                }
                prevStackUp = gamepad2.dpad_up;
                prevStackDown = gamepad2.dpad_down;
                // Go all the way up or down
                if (gamepad2.cross) intake.goToStackPosition(0);
                if (gamepad2.triangle) intake.goToStackPosition(5);
            }
            else intake.off();

            if (gamepad2.square) intake.goToVertical();

            // DRONE LAUNCHER CONTROL
            // Require pressing two keys at once to reduce the chance of accidentally shooting it
            if (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8) launcher.release();
            // Could get rid of the else statement but it'll be useful for testing
            // so we don't have to restart the program every time
            else launcher.hold();

            // TOGGLE CLIMBING
            if ((gamepad2.left_bumper && gamepad2.right_bumper) && !prevClimbingInput){
                isClimbing = !isClimbing;
                // Do this to prevent crashing by running to a pos before setting one
                climber.setTargetPos(climber.getPos());
            }
            prevClimbingInput = gamepad2.left_bumper && gamepad2.right_bumper;

            // TELEMETRY
            //telemetry.addData("Using pixel sensors", usePixelSensors);
            //telemetry.addData("Climbing", isClimbing);
            if (displayDebugTelemetry) {
                telemetry.addLine();
                //telemetry.addData("Driving mode", drivingState);
                //telemetry.addData("Target heading", headingController.getTargetPosition());
                telemetry.addData("Heading", drive.getHeading());
                telemetry.addData("Heading error", headingController.getLastError());
                telemetry.addData("Funny corrected heading", (hasCalibratedFC ? drive.getHeading() : getCorrectedSteeringHeading()));
                telemetry.addData("Driving timer", drivingTimer.seconds());
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
            telemetry.update();
        } // End of the loop

        if (isStopRequested()){
            // Open both grippers if you stop the program, makes it less work for us extracting pixels
            arm.setBothGrippersState(false);
            // The only time I'll ever use sleep()
            sleep(1000);
        }
    }

    boolean hadAnyPixelsWhenPremoved;
    // The big one
    void updateScoringMech(){
        switch (scoringState){
            case INTAKING:
                // Update arm
                // Only poll the sensors we need when we need them to reduce loop times
                arm.updateSensors(true, true, false);
                // Move the arm to the intake, duh
                // Prevent arm hitting stuff near the intake because we swapped it for a speed
                if (Utility.withinErrorOfValue(lift.getHeight(), 0, 2)){
                    arm.pivotGoToIntake();
                } else {
                    arm.preMove();
                }
                // Make sure the deposit doesn't crash into stuff
                arm.centerSteer();
                // Wait to retract the lift until the arm is safely away from the board
                if (pivotTimer.milliseconds() > SteeringArm.pivotAwayFromBordTime) {
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
                // Reset this too
                dontRetractThisTime = false;
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
                if (gripperTimer.milliseconds() > SteeringArm.gripperActuationTime){
                    scoringState = ScoringState.PREMOVED;
                    doubleTapTimer.reset();
                }
                break;

            case PREMOVED:
                arm.preMove();
                arm.centerSteer();
                // Wait to retract the lift until the arm is safely away from the board
                if (pivotTimer.milliseconds() > SteeringArm.pivotAwayFromBordTime) lift.retract();
                // Just make sure we're still holding on
                arm.setBothGrippersState(true);
                arm.setStopperState(false);
                // Toggle the intake off to prevent sucking in pixels when the arm isn't there
                intake.forceToggleOff();
                // Spit out just a little to avoid dragging a third under the tubing
                // Using the gripper timer for this is hacky but oh well
                if (gripperTimer.milliseconds() < (SteeringArm.gripperActuationTime + 200)) intake.reverse();
                else intake.off();
                // Reset poker
                poking = false;
                // Lift up the intake so it's less likely to get crunched
                intake.goToVertical();

                // Switch states when bumper pressed
                // Don't go to scoring if the arm has just been premoved though, this happens when the automatic raises it but the driver
                // tries to manually raise it at the same time. This timer prevents that.
                if (!prevLiftInput && gamepad2.right_bumper && doubleTapTimer.milliseconds() > 450){
                    scoringState = ScoringState.SCORING;
                    // Save this info to prevent it from going down right away if you have nothing
                    hadAnyPixelsWhenPremoved = (arm.pixelIsInBottom() || arm.pixelIsInTop());
                    pivotTimer.reset();
                }
                // Go back to intaking if the arm pulled up before getting both pixels
                if (gamepad2.dpad_left){
                    scoringState = ScoringState.INTAKING;
                }
                break;

            case SCORING:
                // Update arm
                // Only poll the sensors we need when we need them to reduce loop times
                arm.updateSensors(true, false, false);
                // Move it up
                arm.pivotScore();
                lift.extend();
                // Release the top and bottom individually if we wish
                if (gamepad1.left_trigger > 0.5) {
                    arm.setBottomGripperState(false);
                    poking = false;
                }
                if (gamepad1.left_bumper) {
                    arm.setBothGrippersState(false);
                    poking = false;
                }

                // Toggle the poker
                if (gamepad2.b && !prevPokingInput){
                    poking = !poking;
                }
                prevPokingInput = gamepad2.b;
                arm.setStopperState(poking);

                // Press this button to stop the lift from autoRetracting before it autoRetracts.
                // Used when we want to to poke pixels after placing some to avoid the lift going down and back up again.
                if (gamepad2.y) dontRetractThisTime = true;

                // Wait for the arm to move a bit so the deposit doesn't hit the bot
                if (pivotTimer.seconds() > 0.25) arm.updateSteer(hasCalibratedFC ? drive.getHeading() : getCorrectedSteeringHeading());

                // Switch states when the bumper is pressed or both pixels are gone if autoRetract is on
                if (usePixelSensors && (!arm.getTopGripperState() && !arm.getBottomGripperState())){
                    scoringState = ScoringState.SLIDING_UP;
                }
                if (!prevLiftInput && gamepad2.right_bumper){
                    scoringState = ScoringState.BUMPING_UP;
                    // Bump up
                    lift.setExtendedPos(lift.getExtendedPos() + 2);
                }
                // Go back to premoved if we wish
                if (gamepad2.dpad_left){
                    scoringState = ScoringState.PREMOVED;
                    pivotTimer.reset();
                }
                break;

            case SLIDING_UP:
                arm.updateSensors(true, false, false);
                // Raise the lift up gradually to get clear of pixels
                lift.setExtendedPos(lift.getExtendedPos() + 0.15);
                lift.extend();
                // Add bumper to escape as well because we've had bugs with it staying up forever
                if ((!arm.pixelIsInTop() && !arm.pixelIsInBottom()) || gamepad2.right_bumper){
                    scoringState = ScoringState.BUMPING_UP;
                    // Bump up
                    lift.setExtendedPos(lift.getExtendedPos() + 2);
                }
                break;

            case BUMPING_UP:
                lift.extend();
                // Once it's gone up enough, switch states and retract
                // Add bumper to escape as well because we've had bugs with it staying up forever
                if (Utility.withinErrorOfValue(lift.getHeight(), lift.getExtendedPos(), 0.5) || gamepad2.right_bumper) {
                    // Reset that back to normal because we temporarily changed it
                    lift.setExtendedPos(lift.getExtendedPos() - 2);
                    scoringState = ScoringState.INTAKING;
                    // Reset timer so the clock ticks on the arm being away from the board
                    pivotTimer.reset();
                }
        }
        prevLiftInput = gamepad2.right_bumper;
        lift.update();

        // Keep this at 0 until climbing mode is on
        climberTimer.reset();
        // Reset the lift height that the climber will go to
        Climber.targetLiftHeight = Climber.hangingHeight;
    }

    void updateClimibingSystem(){
        // CLIMBER CONTROL
        // Climbing mode moves the arm out of the way, escapes all the pid stuff and just runs things with raw power
        if(isClimbing) {
            switch (climbingState) {
                case REDUCE_SLACK:
                    arm.pivotGoToIntake();
                    arm.setStopperState(false);
                    climber.setPower(-1);
                    lift.setHeight(Climber.targetLiftHeight);
                    lift.update();

                    if (climberTimer.seconds() > climberSlackPullTime) {
                        climber.setPower(0);
                        climber.setTargetPos(climber.getPos());
                        climbingState = ClimbingState.HOLD;
                    }
                    break;

                case HOLD:
                    climber.goToTargetPos();
                    lift.update();
                    // Move it with the right stick if something isn't right (normally this isn't necessary)
                    //Climber.targetLiftHeight += (Climber.targetLiftHeight + 0.2 * -gamepad2.right_stick_y);
                    //lift.setHeight(Climber.targetLiftHeight);

                    if (!(gamepad2.left_stick_y == 0)) {
                        climbingState = ClimbingState.CLIMB;
                    }
                    break;

                case CLIMB:
                    climber.setPower(-gamepad2.left_stick_y);
                    // Lift things
                    // Let it coast and be pulled up if
                    lift.setRawPowerDangerous(0);
                    resetLiftController();
                    // Update so we can get the lift's position
                    lift.update(false);

                    if (gamepad2.left_stick_y == 0) {
                        Climber.targetLiftHeight = lift.getHeight();
                        lift.setHeight(Climber.targetLiftHeight);
                        climber.setTargetPos(climber.getPos());
                        climbingState = ClimbingState.HOLD;
                    }
                    break;
            }
        }
        else{
            lift.retract();
            arm.pivotGoToIntake();
        }
    }

    void resetHeadingController(){
        headingController = new PIDFController(headingCoeffs);
        headingController.setTargetPosition(drive.getHeading());
    }
    void resetLiftController(){
        lift.setCoefficients(Lift.coeffs);
    }

    double getCorrectedSteeringHeading(){
        boolean ver = false;
        if (ver) {
            // Try to fix steering deposit calibration being wrong
            double in = drive.getHeading();
            double chunk;
            double correctedHeading = in;
            if (Math.abs(in) > (2 * Math.PI)) {
                chunk = in % (2 * Math.PI);
                if (in < 0) {
                    correctedHeading = -chunk;
                } else {
                    correctedHeading = chunk;
                }
            }
            return correctedHeading;
        } else {
            // Try to fix steering deposit calibration being wrong
            double in = drive.getHeading();
            double correctedHeading = Math.acos(Math.cos(in));
            if (Math.sin(in) < 0){
                return -correctedHeading;
            } else {
                return correctedHeading;
            }
        }
    }
}
