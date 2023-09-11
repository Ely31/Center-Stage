package org.firstinspires.ftc.teamcode.util.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(group="test")
public class GamepadDebugger extends LinearOpMode {
    // Pre-init
    Gamepad prevGamepad = new Gamepad();
    Gamepad currentGamepad = new Gamepad();
    @Override
    public void runOpMode() {
        // Init
        boolean showBooleans = false;
        telemetry.setMsTransmissionInterval(100);
        waitForStart();
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            // See https://gm0.org/en/latest/docs/software/gamepad.html#storing-gamepad-state
            try{
                prevGamepad.copy(currentGamepad);
                currentGamepad.copy(gamepad1);
            } catch (RobotCoreException ignored){
                // Idk why it wants a try catch statement
            }

            if (currentGamepad.share && !prevGamepad.share){
                showBooleans = !showBooleans;
            }

            // Rumble the left side when dpad left is pressed
            if (gamepad1.dpad_right) gamepad1.rumble(0,1,300);
            // Rumble the right side when right is pressed
            if (gamepad1.dpad_left) gamepad1.rumble(1,0,300);

            if (gamepad1.cross) gamepad1.setLedColor(0,0,255, 300);
            if (gamepad1.circle) gamepad1.setLedColor(255,0,0, 300);
            if (gamepad1.triangle) gamepad1.setLedColor(0,255,0, 300);
            if (gamepad1.square) gamepad1.setLedColor(255,0,255, 300);


            telemetry.addLine("Press share to toggle display of boolean inputs");
            telemetry.addData("at rest", currentGamepad.atRest());
            if (showBooleans) {
                telemetry.addData("A", currentGamepad.a);
                telemetry.addData("B", currentGamepad.b);
                telemetry.addData("Y", currentGamepad.y);
                telemetry.addData("X", currentGamepad.x);
                telemetry.addData("D Down", currentGamepad.dpad_down);
                telemetry.addData("D Up", currentGamepad.dpad_up);
                telemetry.addData("D Left", currentGamepad.dpad_left);
                telemetry.addData("D Right", currentGamepad.dpad_right);
                telemetry.addData("Left Bumper", currentGamepad.left_bumper);
                telemetry.addData("Right Bumper", currentGamepad.right_bumper);
                telemetry.addData("Left stick button", currentGamepad.left_stick_button);
                telemetry.addData("Right stick button", currentGamepad.right_stick_button);
                telemetry.addData("Share", currentGamepad.share);
                telemetry.addData("Start", currentGamepad.start);
                telemetry.addData("Touchpad pressed", currentGamepad.touchpad);
            }

            telemetry.addLine("Left stick: ")
                    .addData("X", currentGamepad.left_stick_x)
                    .addData("Y", currentGamepad.left_stick_y);
            telemetry.addLine("Right stick: ")
                    .addData("X", currentGamepad.right_stick_x)
                    .addData("Y", currentGamepad.right_stick_y);

            telemetry.addData("Left trigger", currentGamepad.left_trigger);
            telemetry.addData("Right trigger", currentGamepad.right_trigger);

            telemetry.addLine("Touchpad finger 1: ")
                    .addData("X",currentGamepad.touchpad_finger_1_x)
                    .addData("Y",currentGamepad.touchpad_finger_1_y);
            telemetry.addLine("Touchpad finger 2: ")
                    .addData("X",currentGamepad.touchpad_finger_2_x)
                    .addData("Y",currentGamepad.touchpad_finger_2_y);

            telemetry.update();
        }
    }
}
