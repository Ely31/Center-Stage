package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Utility;

@Config
@TeleOp(group="test")
public class MotorTestPID extends LinearOpMode {
    // Pre-init
    DcMotor testMotor;
    Utility utility = new Utility();

    public static String motorName = "carousel";
    public static double gearboxRatio = 3.7;
    public static double externalGearRatio = 1.0/1.0;
    double ticksPerRev = gearboxRatio * 28 /externalGearRatio; // This equals the ticks per rev on the final output of the system
    double ticksPerDegree = (ticksPerRev / 360.0);

    public static PIDCoefficients coeffs = new PIDCoefficients(0,0,0);
    PIDFController controller = new PIDFController(coeffs);

    public static double targetAngle = 0;

    public static double minAngle = 0;
    public static double maxAngle = 720;
    public static double angleOne = 200;
    public static double angleTwo = 400;

    @Override
    public void runOpMode() {
        // Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // Graph stuff on dashboard

        testMotor = hardwareMap.dcMotor.get(motorName);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("WARNING: putting in the wrong numbers could be dangerous, depending on the mechanism");
        telemetry.addLine("Use FTC Dash to tune PID");
        telemetry.update();

        waitForStart();
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop

            if (gamepad1.a) targetAngle = minAngle;
            if (gamepad1.b) targetAngle = maxAngle;
            if (gamepad1.x) targetAngle = angleOne;
            if (gamepad1.y) targetAngle = angleTwo;

            // Stop motor by pressing A (it's an emergency stop basically)
            if (!gamepad1.a) runToAngle(targetAngle);
            else testMotor.setPower(0);

            // Telemtry
            telemetry.addData("Ticks per rev", ticksPerRev);
            telemetry.addData("Ticks per degree", ticksPerDegree);
            telemetry.addData("Power", testMotor.getPower());
            telemetry.addData("Target positon in ticks", controller.getTargetPosition());
            telemetry.addData("Position in ticks", testMotor.getCurrentPosition());
            telemetry.addData("Position in degrees", testMotor.getCurrentPosition() / ticksPerDegree);
            telemetry.addData("Error", controller.getLastError());
            telemetry.update();
        }
    }
    void runToAngle(double angle) {
        controller.setTargetPosition((int) (utility.clipValue(minAngle, maxAngle, angle)));
        testMotor.setPower(controller.update(testMotor.getCurrentPosition() / ticksPerDegree));
    }
}
