package org.firstinspires.ftc.teamcode.old;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@Disabled
@Config
@TeleOp
public class OldTeleop extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();
    // Hardware
    TeleMecDrive drive;
    double drivingSpeedMultiplier;
    Arm arm;
    ElapsedTime clawActuationTimer = new ElapsedTime();
    Lift lift;

    // Other variables
    boolean prevClawInput = false;
    // Claw fsm enum
    enum ClawState {
        OPEN,
        ClOSED,
        WAITING_OPEN
    }
    ClawState clawState = ClawState.OPEN;
    // Enable or disable the v4b automatically moving vertical after shutting the claw
    // to save time while scoring and prevent dragging cones on the ground
    public static boolean premoveV4bEnabled  = true;
    enum PremoveState {
        WAITING,
        DONE
    }
    PremoveState premoveState = PremoveState.DONE;

    boolean extended = false;
    boolean prevExtendedInput = false;

    int activeJunction = 2; // 0,1,2,3 is ground, low, medium, and high respectively
    public static double posEditStep = 0.15;
    public static double retractedPosEditStep = 0.07;
    // Telemetry options
    public static boolean debug = true;
    public static boolean instructionsOn = false;

    @Override
    public void runOpMode(){
        // Init
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.2);
        arm = new Arm(hardwareMap);
        lift  = new Lift(hardwareMap);

        waitForStart();
        matchTimer.reset();
        clawActuationTimer.reset();
        while (opModeIsActive()){
            // Send signals to drivers when endgame approaches
           timeUtil.updateAll(matchTimer.milliseconds(), gamepad1, gamepad2);

            // Relate the max speed of the bot to the height of the lift to prevent tipping
            drivingSpeedMultiplier = 1 - (lift.getHeight() * 0.035);
            // Drive the bot
            drive.driveFieldCentric(
                    gamepad1.left_stick_x * drivingSpeedMultiplier,
                    gamepad1.left_stick_y * drivingSpeedMultiplier,
                    gamepad1.right_stick_x * drivingSpeedMultiplier * 0.8,
                    gamepad1.right_trigger);
            if (gamepad1.share) drive.resetHeading();

            // CLAW CONTROL
            updateClaw(gamepad1.left_bumper);

            // ARM AND LIFT CONTROL
            // Edit things
            // Switch active junction using the four buttons on gamepad one
            if (gamepad1.cross) activeJunction = 0;
            if (gamepad1.square) activeJunction = 1;
            if (gamepad1.triangle) activeJunction = 2;
            if (gamepad1.circle) activeJunction = 3;
            // Edit the current level with the dpad on gamepad two
            if (gamepad2.dpad_up) lift.editCurrentPos(activeJunction, posEditStep);
            if (gamepad2.dpad_down) lift.editCurrentPos(activeJunction, -posEditStep);
            // Edit retracted pos for grabbing off the stack (this may be a scuffed way of doing it but comp is in two days)
            if (gamepad2.triangle) lift.editRetractedPos(retractedPosEditStep);
            if (gamepad2.cross) lift.editRetractedPos(-retractedPosEditStep);

            // Rising edge detector controlling a toggle for the extended state
            if ((gamepad1.left_trigger > 0) && !prevExtendedInput) {
                extended = !extended;
                // Extend or retract the lift based on this
                if (extended) extend(); else retract();
            }
            prevExtendedInput = (gamepad1.left_trigger > 0);

            // Update the lift, very important
            lift.update();


            // Print stuff to telemetry if we want to
            if (debug) {
                telemetry.addData("extended", extended);
                telemetry.addData("claw state", clawStateToString());
                lift.disalayDebug(telemetry);
                arm.displayDebug(telemetry);
                telemetry.addData("avg loop time (ms)", timeUtil.getAverageLoopTime());
                telemetry.addData("period", timeUtil.getPeriod());
                telemetry.addData("time", matchTimer.seconds());
            }
            // Someone should be able to learn how to drive without looking at the source code
            if (instructionsOn) {
                telemetry.addLine();
                telemetry.addLine();
                telemetry.addLine("Gamepad 1 controls:");
                telemetry.addLine("Driving: Left stick is translation, right stick x is rotation. Use the right trigger to slow down.");
                telemetry.addLine("calibrate feild-centric with the share button after you point the bot with the claw facing towards you");
                telemetry.addLine();
                telemetry.addLine("Lift and arm: toggle the claw open and closed with the left bumper, and toggle extend/retract with the left trigger");
                telemetry.addLine("Switch levels with buttons cross, square, triangle, and circle.");
                telemetry.addLine("Ground level is cross, levels get higher as you travel clockwise along the four buttons");
                telemetry.addLine();
                telemetry.addLine("Gamepad 2:");
                telemetry.addLine("make fine adjustments to the current height with dpad up and down.");
                telemetry.addLine("The height tweaks made are saved and will persist until the opmode is stopped.");
            }
            telemetry.update();
        } // End of the loop
    }

    // Methods
    void extend(){
        lift.goToJunction(activeJunction);
        // Have a special case for the gronud junction
        if (activeJunction == 0){
            arm.scoreGroundPassthrough();
        } else arm.scorePassthrough();
    }
    void retract(){
        lift.retract();
        arm.grabPassthrough();
    }

    void updateClaw(boolean input){
        switch(clawState){
            case OPEN:
                arm.openClaw();
                // If the button is pressed for the first time or the sensor detects a cone, close the claw
                if ((input && !prevClawInput) || arm.coneIsInClaw()){
                    clawState = ClawState.ClOSED;
                }
                break;
            case ClOSED:
                arm.closeClaw();
                if (input && !prevClawInput){
                    clawState = ClawState.WAITING_OPEN;
                }
                break;
            case WAITING_OPEN:
                arm.openClaw();
                // If you press the button again, close it
                if (input && !prevClawInput){
                    clawState = ClawState.ClOSED;
                }
                // If it stops seeing the cone, go back to the open state, where it starts to look for one again
                if (!arm.coneIsInClaw()){
                    clawState = ClawState.OPEN;
                }
                break;
        }
        prevClawInput = input;
    }

    String clawStateToString(){
        // I feel like this should be easier
        String output;
        switch (clawState){
            case OPEN:
                output = "open";
                break;
            case ClOSED:
                output = "closed";
                break;
            case WAITING_OPEN:
                output = "waiting open";
                break;
            default:
                output = "default";
                break;
        }
        return output;
    }

}
