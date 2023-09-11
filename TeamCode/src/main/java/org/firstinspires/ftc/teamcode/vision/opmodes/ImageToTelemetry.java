package org.firstinspires.ftc.teamcode.vision.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.vision.workspace.ImageToTelemetryPipeline;
import org.firstinspires.ftc.teamcode.vision.workspace.ImageToTelemetryPipeline2;

@TeleOp(group = "test")
public class ImageToTelemetry extends LinearOpMode {
    // Pre-init
    Camera camera;
    ImageToTelemetryPipeline2 pipeline = new ImageToTelemetryPipeline2();
    ElapsedTime processImageThrottle = new ElapsedTime();

    int refreshRate = 300; // In milliseconds

    @Override
    public void runOpMode() {
        // Init
        camera = new Camera(hardwareMap, pipeline);
        // Set up telemetry right
        telemetry.setMsTransmissionInterval(refreshRate);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();
        // Pre-run
        processImageThrottle.reset();
        while (opModeIsActive()) {
            // Autonomous instructions
            if (processImageThrottle.milliseconds() > refreshRate) {
                // Print to telemetry once the throttle time is up
                pipeline.toTelemetry(telemetry);
                telemetry.update();
                // Reset the timer so it loops
                processImageThrottle.reset();
            }
        }
    }
}
