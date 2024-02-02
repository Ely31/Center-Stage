package org.firstinspires.ftc.teamcode.vision.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.vision.workspace.TeamPropDetector2;

@TeleOp(group = "test")
public class PropPipelineTest extends LinearOpMode {

    Camera camera;
    TeamPropDetector2 pipeline = new TeamPropDetector2();

    boolean prevInput = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(100);
        camera = new Camera(hardwareMap, pipeline);
    waitForStart();

    while (opModeIsActive()){
        telemetry.addData("Position", pipeline.getAnalysis());
        telemetry.update();
        }
    }
}
