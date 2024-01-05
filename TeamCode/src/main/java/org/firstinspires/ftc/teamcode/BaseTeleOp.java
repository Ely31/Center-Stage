package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@TeleOp(group = "test")
public class BaseTeleOp extends LinearOpMode {
    // Pre init
    TeleMecDrive drive;
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode(){
        // Init
        //PhotonCore.enable();
        telemetry.setMsTransmissionInterval(100);
        drive = new TeleMecDrive(hardwareMap, 0.4, false);

        waitForStart();
        timer.reset();
        while (opModeIsActive()){
            // Send signals to drivers when endgame approaches
            timeUtil.update(timer.milliseconds());
            timeUtil.updateGamepads(gamepad1, gamepad2);

            drive.driveFieldCentric(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.right_trigger);

            if (gamepad1.back) drive.resetIMU();

            telemetry.addData("avg loop time (ms)", timeUtil.getAverageLoopTime());
            telemetry.addData("period", timeUtil.getPeriod());
            telemetry.addData("time", timer.seconds());
            telemetry.update();
        }
    }
}