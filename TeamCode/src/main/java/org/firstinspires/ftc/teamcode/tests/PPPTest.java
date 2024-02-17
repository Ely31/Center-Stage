package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ExtendoIntake;

@TeleOp(name="",group="test")
public class PPPTest extends LinearOpMode {
    // Pre-init
    ExtendoIntake intake;

    enum State {
        CHILLIN,
        S2,
        S3,
        S4,
        DONE
    }
    State state = State.CHILLIN;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Init
        intake = new ExtendoIntake(hardwareMap);

        waitForStart();
        timer.reset();
        // Pre-run
        while (opModeIsActive()) {
            // TeleOp loop
            switch (state){
                case CHILLIN:
                    intake.goToVertical();
                    timer.reset();
                    intake.off();
                    if (gamepad1.b){state = State.S2; timer.reset();}
                    break;
                case S2:
                    intake.gotoRawPosition(0.33);
                    if (timer.milliseconds() > 800) {
                        state = State.S3;
                        timer.reset();
                    }
                    break;
                case S3:
                    intake.on(false, 0.3);
                    if (timer.milliseconds() > 50){
                        intake.off();
                        state = State.S4;
                        timer.reset();
                    }
                    break;
                case S4:
                    intake.goToVertical();
                    if (timer.milliseconds() > 300){
                        state = State.DONE;
                        timer.reset();
                    }
                    break;
                case DONE:
                    if (gamepad1.b) {state = State.CHILLIN;}
                    break;
            }

            intake.displayDebug(telemetry);
            telemetry.addData("timer", timer.milliseconds());
            telemetry.addData("state", state.name());
            telemetry.update();
        }
    }
}
