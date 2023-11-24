package org.firstinspires.ftc.teamcode.messing;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@SuppressLint("DefaultLocale")
// File writing code stolen from https://discord.com/channels/225450307654647808/225451520911605765/936127511086116924
public class AutoTrainerAdv {
    StandardTrackingWheelLocalizer localizer;
    Telemetry telemetry;
    ElapsedTime timer;
    Pose2d lastPose = new Pose2d();
    Pose2d startPose;

    Pose2d lastCapturedPose;

    public int poseIndex = 0;
    public String latestPathSegment = "";
    String trajSequenceString;

    final String filePath = "/storage/self/primary/";
    File file = new File(filePath + "PoseSequence.csv");
    BufferedWriter writer = null;

    // Constructor
    public AutoTrainerAdv(HardwareMap hwmap, Telemetry telemetry, boolean isWingSide){
        this.localizer = new StandardTrackingWheelLocalizer(hwmap);
        this.telemetry = telemetry;
        if (isWingSide) startPose = new Pose2d(-35.25, -61.5, Math.toRadians(90));
        else startPose = new Pose2d(11.75, -61.5, Math.toRadians(90));
        lastCapturedPose = startPose;
        trajSequenceString = "TrajectorySequence generatedTrajectory = drive.trajectorySequenceBuilder(" + poseToString(startPose) + ")\n";
        localizer.setPoseEstimate(startPose);
        timer = new ElapsedTime();
        initializeWriter();
    }
    // Called in init in the constructor
    void initializeWriter(){
        try {
            writer = new BufferedWriter(new FileWriter(file));
            writer.write("time taken after running, pose, code");
            writer.write("\n");

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    // Call this when the program is stopped
    public void closeWriter(){
        // Finish up the trajsequence string
        trajSequenceString += ".build();";
        // And stick it on the end of the file
        try{
        writer.write("\n\n\n");
        writer.write(String.format("%.1f", timer.seconds()) + "," + "\"" + trajSequenceString + "\"");

        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            writer.flush();
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void update(){
        localizer.update();
        lastPose = localizer.getPoseEstimate();
        telemetry.addData("current pose", poseToString(lastPose));
        telemetry.addData("trajSequenceString", trajSequenceString);
    }


    public String poseToString(Pose2d pose){
        return String.format("new Pose2d(%.2f, %.2f, Math.toRadians(%.1f))", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }


    public void addPoseToFile(boolean pathType, double endTangent){
        // Tell it we've just added a new pose
        poseIndex ++;
        // Now the normal stuff that happens every time
        // True is splineToSpline, false is lineToSpline
        if (pathType) {
            // Tack on another splineToSpline to the big auto-generated code chunk
            latestPathSegment = ".splineToSplineHeading(" + poseToString(lastPose)
                    + ", Math.toRadians(" + String.format("%.1f", Math.toDegrees(endTangent)) + "))";
        } else {
            latestPathSegment = ".lineToSplineHeading(" + poseToString(lastPose) + ")";
        }
        try {
            // Add quotes (the \" parts) because the pose has commas in it that would mess up the csv file otherwise
            writer.write(
                    String.format("%.2f", timer.seconds()) + ",\"" + poseToString(lastPose) + "\"" + ",\"" + latestPathSegment + "\"" + "\n");
        } catch (IOException e) {
            e.printStackTrace();
        }

        trajSequenceString += latestPathSegment;
        lastCapturedPose = lastPose;
    }
}
