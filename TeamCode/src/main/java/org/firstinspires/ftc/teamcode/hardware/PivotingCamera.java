package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class PivotingCamera {

    public OpenCvWebcam webcam;
    private Servo servo;

    // Servo angle constants
    public static double pos0 = 0;
    public static double pos1 = 0.2;
    public static double pos2 = 0.24;
    public static double pos3 = 0.28;

    public static double offset = 0;

    double[] posArray = {pos0, pos1, pos2, pos3};

    public PivotingCamera(HardwareMap hwmap){
        init(hwmap);
    }
    public PivotingCamera(HardwareMap hwmap, OpenCvPipeline pipeline){
        init(hwmap);
        setPipeline(pipeline);
    }

    public void init(HardwareMap hwmap) { // Initialization of camera and pipeline
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwmap.get(WebcamName.class, "Webcam 1"));
        servo = hwmap.get(Servo.class, "cameraPivot");
        servo.setDirection(Servo.Direction.REVERSE);
        setJunction(0);

        webcam.setMillisecondsPermissionTimeout(1000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
    }

    public void stopStreaming(){
        webcam.stopStreaming();
    }
    public void setPipeline(OpenCvPipeline pipeline){
        webcam.setPipeline(pipeline);
    }


    // Pivot servo stuff
    public void setAngle(double angle){
        servo.setPosition(angle + offset);
    }
    public void setJunction(int junction){
        setAngle(posArray[junction]);
    }
}