package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="",group="test")
public class PrimitiveServoTest extends LinearOpMode {
    // Pre-init
    Servo servo;

    public static double pos1 = 0;
    public static double pos2 = 1;

    @Override
    public void runOpMode() {
        // Init
        // Must be named "testservo" in the config
        servo = hardwareMap.get(Servo.class, "testservo");
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            // Only if you press the trigger does it control the servo
            if (!(gamepad1.left_trigger == 0)) servo.setPosition(gamepad1.left_trigger);

            if (gamepad1.a) {
                servo.setPosition(pos1);
            } else if (gamepad1.b){
                servo.setPosition(pos2);
            }
        }
    }
}
