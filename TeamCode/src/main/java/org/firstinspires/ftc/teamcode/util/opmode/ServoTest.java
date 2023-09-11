package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group="test")
public class ServoTest extends LinearOpMode {
    // Pre-init
    public static double Pos1 = 0;
    public static double Pos2 = 1;
    public static double PosCenter = (Pos1+Pos2/2);

    public static String servoName = "test";

    Servo testServo;

    @Override
    public void runOpMode() {
        // Init
        testServo = hardwareMap.servo.get(servoName);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("BE CAREFUL! it's possible to break something by typing in the wrong numbers");
        telemetry.update();

        waitForStart();
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            if (gamepad1.x) testServo.setPosition(Pos1);
            if (gamepad1.b) testServo.setPosition(Pos2);
            if (gamepad1.a) testServo.setPosition(PosCenter);

            PosCenter = (Pos1+Pos2/2);

            telemetry.addData("target pos", testServo.getPosition());
            telemetry.addData("pos1", Pos1);
            telemetry.addData("pos2", Pos2);
            telemetry.addData("posCenter", PosCenter);
            telemetry.update();
        }
    }
}
