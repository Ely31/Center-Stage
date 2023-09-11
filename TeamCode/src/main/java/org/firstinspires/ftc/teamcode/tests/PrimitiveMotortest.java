package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="",group="test")
public class PrimitiveMotortest extends LinearOpMode {
    // Pre-init
    DcMotor motor;
    @Override
    public void runOpMode() {
        // Init
        // This motor must be configured as "testmotor"
        motor = hardwareMap.get(DcMotor.class, "testmotor");
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            motor.setPower(-gamepad1.left_stick_y);

            if (gamepad1.a) {
                motor.setPower(1);
            } else if (gamepad1.b) {
                motor.setPower(-1);
            }
        }
    }
}
