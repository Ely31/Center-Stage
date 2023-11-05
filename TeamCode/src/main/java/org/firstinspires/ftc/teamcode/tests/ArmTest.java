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

            arm.setBothGrippersState(gamepad1.x);

            if (gamepad1.dpad_left) arm.pivotGoToIntake();
            else if (gamepad1.dpad_right) arm.pivotScore();
           /*
            else {
                arm.setPivotPos(gamepad1.right_trigger+.05);
                arm.setEndPos(gamepad1.left_trigger);
            }*/

            telemetry.addData("Pivot Pos", arm.getPivotPos());
            telemetry.addData("End Pos", arm.getPivotPos());
            telemetry.update();
        }
    }
}
