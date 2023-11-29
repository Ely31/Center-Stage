package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;

@TeleOp(name="",group="test")
public class ArmTest extends LinearOpMode {
    // Pre-init
    Arm arm;
    @Override
    public void runOpMode() {
        // Init
    arm = new Arm(hardwareMap);

        waitForStart();
        // Pre-run
        while (opModeIsActive()) {
            // TeleOp loop

            //arm.setBothGrippersState(gamepad1.x);

            if (gamepad1.dpad_left) arm.pivotGoToIntake();
            else if (gamepad1.dpad_right) arm.pivotScore();

            if (gamepad1.a) arm.setTopGripperState(true);
            if (gamepad1.b) arm.setTopGripperState(false);
            if (gamepad1.x) arm.setBottomGripperState(true);
            if (gamepad1.y) arm.setBottomGripperState(false);

            if (gamepad1.left_bumper) arm.setStopperState(false);
            if (gamepad1.right_bumper) arm.setStopperState(true);

            arm.update();

            arm.displayDebug(telemetry);
            telemetry.update();
        }
    }
}
