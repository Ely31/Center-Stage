package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DrivingInstructions;
import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.ScoringMech;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@Config
@TeleOp
public class Teleop2 extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();
    TeleMecDrive drive;
    double drivingSpeedMultiplier;
    ScoringMech scoringMech;

    ElapsedTime clawActuationTimer = new ElapsedTime();
    ElapsedTime pivotActuationTimer = new ElapsedTime();

    // Claw fsm enum
    enum GrabbingState {
        OPEN,
        ClOSED,
        WAITING_OPEN
    }
    GrabbingState grabbingState = GrabbingState.OPEN;

    enum ScoringState{
        RETRACTED,
        PREMOVED,
        SCORING
    }
    ScoringState scoringState = ScoringState.RETRACTED;

    public static boolean autoRetract = true;

    // Stuff for rising edge detectors
    boolean prevClawInput = false;
    boolean prevScoringInput = false;

    boolean prevStackIndexUpInput = false;
    boolean prevStackIndexDownInput = false;

    boolean hackyExtendSignal = false;

    // Lift constants
    public static double liftPosEditStep = 0.15;
    public static double liftRawPowerAmount = 0.2;

    // Telemetry options
    public static boolean debug = true;
    public static boolean instructionsOn = true;

    @Override
    public void runOpMode(){
        // Init
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.4);
        scoringMech = new ScoringMech(hardwareMap);

        waitForStart();
        matchTimer.reset();
        clawActuationTimer.reset();
        pivotActuationTimer.reset();
        while (opModeIsActive()){
            // Send signals to drivers when endgame approaches
           timeUtil.updateAll(matchTimer.milliseconds(), gamepad1, gamepad2);

            // Slow down the bot when scoring
            if (scoringState == ScoringState.SCORING) drivingSpeedMultiplier = 0.3;
            else drivingSpeedMultiplier = 1;
            // Drive the bot
            drive.driveFieldCentric(
                    gamepad1.left_stick_x * drivingSpeedMultiplier,
                    gamepad1.left_stick_y * drivingSpeedMultiplier,
                    gamepad1.right_stick_x * drivingSpeedMultiplier * 0.8,
                    gamepad1.right_trigger);

            // Manually calibrate feild centric with a button
            if (gamepad1.share) drive.resetHeading();

            // ARM AND LIFT CONTROL
            // Edit things
            // Switch active junction using the four buttons on gamepad one
            if (gamepad1.cross)     scoringMech.setActiveScoringJunction(0);
            if (gamepad1.square)    scoringMech.setActiveScoringJunction(1);
            if (gamepad1.triangle)  scoringMech.setActiveScoringJunction(2);
            if (gamepad1.circle)    scoringMech.setActiveScoringJunction(3);

            // This bit is hacked in to change the behavior without ripping up too much code
            // If you press any of the buttons, make the signal true. If not, it's false.
            hackyExtendSignal = gamepad1.cross || gamepad1.square || gamepad1.triangle || gamepad1.circle;

            // Edit the current level with the dpad on gamepad two
            if (gamepad2.dpad_up)   scoringMech.editCurrentLiftPos(liftPosEditStep);
            if (gamepad2.dpad_down) scoringMech.editCurrentLiftPos(-liftPosEditStep);
            // Edit retracted pose for grabbing off the stack using rising edge detectors
            if (gamepad2.triangle && !prevStackIndexUpInput) {
                scoringMech.setStackIndex(scoringMech.getStackIndex()+1);
                scoringMech.setRetractedGrabbingPose(scoringMech.getStackIndex());
            }
            prevStackIndexUpInput = gamepad2.triangle;
            if (gamepad2.cross && !prevStackIndexDownInput) {
                scoringMech.setStackIndex(scoringMech.getStackIndex()-1);
                scoringMech.setRetractedGrabbingPose(scoringMech.getStackIndex());
            }
            prevStackIndexDownInput = gamepad2.cross;

            // Claw control
            updateClaw(gamepad1.left_bumper);

            // Scoring mech (lift and v4b) control
            updateScoringmech();

            // Update the lift so its pid controller runs, very important
            // But, if you press a special key combo, escape pid control and bring the lift down
            // With raw power to fix a lift issue
            if (gamepad2.dpad_left && gamepad2.share){
                scoringMech.setRawLiftPowerDangerous(-liftRawPowerAmount);
                scoringMech.zeroLift();
            } else
            if (gamepad2.dpad_right && gamepad2.share) {
                scoringMech.setRawLiftPowerDangerous(1);
                scoringMech.zeroLift();
            }
            else scoringMech.updateLift();


            // TELEMETRY
            if (debug) {
                telemetry.addData("scoring state", scoringState.name());
                telemetry.addData("active junction", scoringMech.getActiveScoringJunction());
                telemetry.addData("grabbing state", grabbingState.name());
                telemetry.addData("stack index", scoringMech.getStackIndex());
                telemetry.addData("heading", drive.getHeading());
                scoringMech.displayDebug(telemetry);
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

    // Methods for handling the complex interactions of the scoring mech and claw
    void updateClaw(boolean input){
        switch(grabbingState){
            case OPEN:
                scoringMech.openClaw();
                // If the button is pressed for the first time or the sensor detects a cone, close the claw
                if ((input && !prevClawInput) || scoringMech.getConeStatus()){
                    grabbingState = GrabbingState.ClOSED;
                }
                break;
            case ClOSED:
                scoringMech.closeClaw();
                if (input && !prevClawInput){
                    grabbingState = GrabbingState.WAITING_OPEN;
                }
                break;
            case WAITING_OPEN:
                scoringMech.openClaw();
                // If you press the button again, close it
                if (input && !prevClawInput){
                    grabbingState = GrabbingState.ClOSED;
                }
                // If it stops seeing the cone, go back to the open state, where it starts to look for one again
                if (!scoringMech.getConeStatus()){
                    grabbingState = GrabbingState.OPEN;
                }
                break;
        }
        prevClawInput = input;
        // Keep the timer at zero until we want it to start ticking, when the claw is closed
        if (!(grabbingState == GrabbingState.ClOSED)) clawActuationTimer.reset();
    }

    void updateScoringmech(){
        // Run the scoring fsm
        switch (scoringState){
            case RETRACTED:
                scoringMech.retract(pivotActuationTimer.milliseconds());
                // Handle bracer stuff
                if (pivotActuationTimer.milliseconds() > Arm.pivotActuationTime + 500){
                    // The servo doesn't need to be working once everything is back inside the bot
                    scoringMech.extendBracer();
                }
                // Don't retract the bracer when doing ground junctions, it would hit the bot
                else if(!(scoringMech.getActiveScoringJunction() == 0)){
                    scoringMech.retractBracer();
                }

                if (grabbingState == GrabbingState.ClOSED && clawActuationTimer.milliseconds() > Arm.clawActuationTime){
                    scoringState = ScoringState.PREMOVED;
                }
                break;

            case PREMOVED:
                scoringMech.retractPremoved(pivotActuationTimer.milliseconds());

                if (pivotActuationTimer.milliseconds() > Arm.pivotActuationTime + 300){
                    // The servo doesn't need to be working once everything is back inside the bot
                    scoringMech.extendBracer();
                }
                // Don't retract the bracer when doing ground junctions, it would hit the bot
                else if(!(scoringMech.getActiveScoringJunction() == 0)){
                    scoringMech.retractBracer();
                }

                if (((gamepad1.left_trigger > 0) && !prevScoringInput) || hackyExtendSignal) {
                    scoringState = ScoringState.SCORING;
                }
                if (grabbingState == GrabbingState.OPEN || grabbingState == GrabbingState.WAITING_OPEN){
                    scoringState = ScoringState.RETRACTED;
                }
                break;

            case SCORING:
                scoringMech.score();
                scoringMech.extendBracer();
                // Retract if you press the retract button
                if ((gamepad1.left_trigger > 0) && !prevScoringInput) {
                    pivotActuationTimer.reset();
                    scoringState = ScoringState.RETRACTED;
                }

                if (autoRetract) {
                    // Or, retract automatically when you drop the cone
                    if (grabbingState == GrabbingState.OPEN) {
                        pivotActuationTimer.reset();
                        scoringState = ScoringState.RETRACTED;
                    }
                }
                break;
        }
        prevScoringInput = (gamepad1.left_trigger > 0);
    }
}
