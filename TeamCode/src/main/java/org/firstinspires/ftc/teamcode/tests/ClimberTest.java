package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Climber;

@TeleOp(name="",group="test")
public class ClimberTest extends LinearOpMode {
    // Pre-init
    Climber climber;
    @Override
    public void runOpMode() {
        // Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        climber = new Climber(hardwareMap);
        climber.setCoefficients(Climber.coeffs);
        waitForStart();
    
        // Pre-run
        climber.retract();
        while (opModeIsActive()) {
            // TeleOp loop
            if(gamepad1.dpad_left) climber.release();
            else if (gamepad1.dpad_right) climber.hold();

            climber.toggle(gamepad1.left_bumper && gamepad1.right_bumper);

            climber.update();
            climber.disalayDebug(telemetry);
            telemetry.update();
        }
    }
}
