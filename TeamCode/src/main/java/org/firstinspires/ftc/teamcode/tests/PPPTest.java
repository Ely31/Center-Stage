package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.PurplePixelPusher;

@TeleOp(name="",group="test")
public class PPPTest extends LinearOpMode {
    // Pre-init
    PurplePixelPusher ppp;
    @Override
    public void runOpMode() {
        // Init
        ppp = new PurplePixelPusher(hardwareMap);

        waitForStart();
        // Pre-run
        while (opModeIsActive()) {
            // TeleOp loop
            ppp.setState(!gamepad1.a);

            ppp.displayDebug(telemetry);
            telemetry.update();
        }
    }
}
