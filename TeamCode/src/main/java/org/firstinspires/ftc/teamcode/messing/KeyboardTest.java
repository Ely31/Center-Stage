package org.firstinspires.ftc.teamcode.messing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.REDetector;

@TeleOp(group="test")
public class KeyboardTest extends LinearOpMode {
    // Pre-init
    Keyboard keyboard;

    StringBuilder builder = new StringBuilder();

    int cursorPos = 0;

    REDetector cursorLeft = new REDetector();
    REDetector cursorRight = new REDetector();

    @Override
    public void runOpMode() {
        // Init
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        keyboard = new Keyboard(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            // TeleOp loop
            keyboard.update();

            // Type stuff
            if (keyboard.letterTyped()) builder.append(keyboard.getSelectedChar());
            if (keyboard.spacePressed()) builder.append(' ');
            if (keyboard.delPressed()) {
                if (builder.length() > 0){
                    builder.deleteCharAt(builder.length()-1);
                }
            }
            if (keyboard.enterPressed()) builder.append('\n');

            telemetry.addLine(keyboard.getDisplayString());
            telemetry.addLine();
            telemetry.addLine(builder.toString());
            telemetry.update();
        }
    }
}
