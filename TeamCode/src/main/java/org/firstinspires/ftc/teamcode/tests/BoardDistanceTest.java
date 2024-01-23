package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm3;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@Config
//@Photon
@TeleOp(group = "test")
public class BoardDistanceTest extends LinearOpMode {
    TeleMecDrive drive;
    Arm3 arm;

    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();

    PIDFController boardDistanceController;
    public static PIDCoefficients boardCoeffs = new PIDCoefficients(0.05,0.0001,5);

    boolean prevHeadingResetInput = false;

    public static double boardTargetDistance = 15;

    @Override
    public void runOpMode(){
        // Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.3, false);
        arm = new Arm3(hardwareMap);

        boardDistanceController = new PIDFController(boardCoeffs);
        boardDistanceController.setTargetPosition(boardTargetDistance);


        waitForStart();

        // Automatic feild centric calibration
        drive.setHeadingOffset(AutoToTele.endOfAutoHeading + Math.toRadians(-90*AutoToTele.allianceSide));

        // START OF TELEOP LOOP
        while (opModeIsActive()){


            // DRIVING
                // Lock heading with pid controller if you aren't turning
                drive.driveFieldCentric(
                        -boardDistanceController.update(arm.getBoardDistance()),
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        1
                );

            // Manually calibrate field centric with a button
            if (gamepad1.share && !prevHeadingResetInput) {
                drive.resetIMU();
                drive.resetHeadingOffset();
            }
            prevHeadingResetInput = gamepad1.share;

            arm.update(false, false, true);

            telemetry.addData("Board lock .update", boardDistanceController.update(arm.getBoardDistance()));
            telemetry.addData("Board lock error", boardDistanceController.getLastError());
            telemetry.addData("Board lock target pos", boardDistanceController.getTargetPosition());
            telemetry.addLine();
            telemetry.addLine("SUBSYSTEMS");
            telemetry.addLine();
            drive.displayDebug(telemetry);
            arm.displayDebug(telemetry);
            timeUtil.update(matchTimer.milliseconds());
            timeUtil.displayDebug(telemetry, matchTimer);

            telemetry.update();
        } // End of the loop

    }

    void resetBoardDistanceController(){
        boardDistanceController = new PIDFController(boardCoeffs);
        boardDistanceController.setTargetPosition(boardTargetDistance);
    }
}
