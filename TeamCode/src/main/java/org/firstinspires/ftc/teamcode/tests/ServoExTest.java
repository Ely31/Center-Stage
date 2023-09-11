package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.actuators.ServoEx;

@Config
@TeleOp(group = "test")
public class ServoExTest extends LinearOpMode {
    // Pre init
    ServoEx servo;
    public static String name = "intakeRelease";
    public static double travel = 205;
    @Override
    public void runOpMode(){
        // Init
        servo = new ServoEx(hardwareMap, name, travel);
        waitForStart();
        while (opModeIsActive()){
            // Teleop code
            if (gamepad1.a) servo.setAngle(180);
            else if (gamepad1.b) servo.setAngle(0);

            if (gamepad1.back) servo.setDirection(true);
            else if (gamepad1.start) servo.setDirection(false);

            if (!gamepad1.a && !gamepad1.b) servo.setAngle(-gamepad1.left_stick_y * travel);

            telemetry.setMsTransmissionInterval(100);
            telemetry.addData("angle", servo.getAngle());
            telemetry.addData("direction", servo.getDirection());
            telemetry.update();
        }
    }
}