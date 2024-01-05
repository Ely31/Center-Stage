package org.firstinspires.ftc.teamcode.messing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;

@TeleOp(group = "test")
public class AutoTrainerAdvTest extends LinearOpMode {
    // Pre init
    TeleMecDrive teleDrive;
    AutoTrainerAdv trainer;

    boolean lastCaptureLineToInput = false;
    boolean lastCaptureSplineToInput = false;

    @Override
    public void runOpMode(){
        // Init
        telemetry.setMsTransmissionInterval(100);
        teleDrive = new TeleMecDrive(hardwareMap, 0.4, false);
        trainer = new AutoTrainerAdv(hardwareMap, telemetry, true);

        ElapsedTime telemetryMessageTime = new ElapsedTime();

        waitForStart();
        telemetryMessageTime.reset();
        while (opModeIsActive()){

            // Suspend driving if you press a key because I'm too lazy to do the setting of the end tangent properly
            if (gamepad1.left_bumper) teleDrive.driveFieldCentric(0,0,0,0);
            else {
                teleDrive.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);
            }
            if (gamepad1.back) teleDrive.resetIMU();

            // Make a lineTo
            if (!lastCaptureLineToInput && gamepad1.a && gamepad1.left_bumper){
                trainer.addPoseToFile(false, 0);
                telemetryMessageTime.reset();
            }
            lastCaptureLineToInput = gamepad1.a;
            // Make a splineTo
            if (!lastCaptureSplineToInput && gamepad1.y && gamepad1.left_bumper){
                trainer.addPoseToFile(true, stickToAngle(gamepad1));
                telemetryMessageTime.reset();
            }
            lastCaptureSplineToInput = gamepad1.y;

            if (telemetryMessageTime.seconds() < 5){
                telemetry.addLine("Pose " + trainer.poseIndex + " captured");
                telemetry.addLine(trainer.latestPathSegment);
            }

            trainer.update();
            if (gamepad1.left_bumper) telemetry.addData("End tangent setting", stickToAngle(gamepad1));
            telemetry.addLine("Add a lineTo with A and a splineTo with Y");
            telemetry.addLine("Suspend driving with the left bumper to set an endTangent");
            telemetry.update();
        }
        // After the opmode is stopped, close the file writer
        trainer.closeWriter();
    }

    double stickToAngle(Gamepad gamepad){
        return Math.atan2(-gamepad.left_stick_y, gamepad.left_stick_x);
    }
}