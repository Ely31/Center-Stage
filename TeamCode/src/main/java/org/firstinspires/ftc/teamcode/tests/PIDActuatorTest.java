package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.actuators.PIDActuator;

@TeleOp(group = "test")
public class PIDActuatorTest extends LinearOpMode {
    PIDActuator actuator;
    @Override
    public void runOpMode() throws InterruptedException {

        actuator = new PIDActuator(hardwareMap, "carousel", 3.7);
        actuator.setAngularLimits(0,360);
        actuator.setAngleCoefficients(new PIDCoefficients(0.001,0,0));

        waitForStart();
        while (opModeIsActive()){
            actuator.update();

            if (gamepad1.dpad_left) actuator.setAnglePID(0);
            if (gamepad1.dpad_right) actuator.setAnglePID(360);

            actuator.displayDebugInfo(telemetry);
            telemetry.update();
        }
    }
}
