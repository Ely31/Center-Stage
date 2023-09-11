package org.firstinspires.ftc.teamcode.messing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;

@TeleOp
public class AutoTrainerTest extends LinearOpMode {
    // Pre init
    TeleMecDrive teleDrive;
    AutoTrainer trainer;

    boolean lastCapturePoseInput = false;

    @Override
    public void runOpMode(){
        // Init
        telemetry.setMsTransmissionInterval(100);
        teleDrive = new TeleMecDrive(hardwareMap, 0.4);
        trainer = new AutoTrainer(hardwareMap, telemetry, new Pose2d(0,0, Math.toRadians(-90)));

        ElapsedTime telemetryMessageTime = new ElapsedTime();

        waitForStart();
        telemetryMessageTime.reset();
        while (opModeIsActive()){

            teleDrive.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);
            if (gamepad1.back) teleDrive.resetHeading();

            if (!lastCapturePoseInput && gamepad1.a){
                trainer.addPoseToFile();
                telemetryMessageTime.reset();
            }
            lastCapturePoseInput = gamepad1.a;

            if (telemetryMessageTime.seconds() < 1.5){
                telemetry.addLine("Pose " + trainer.poseIndex + " captured");
            }

            trainer.update();
            telemetry.update();
        }
        // After the opmode is stopped, close the file writer
        trainer.closeWriter();
    }
}