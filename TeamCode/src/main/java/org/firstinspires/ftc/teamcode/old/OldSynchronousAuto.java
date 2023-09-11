package org.firstinspires.ftc.teamcode.old;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.AutoScoringMech;
import org.firstinspires.ftc.teamcode.hardware.PivotingCamera;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.vision.workspace.SignalPipeline;

//this autonomous is meant for if you start on the left side of the field
//regular is the red side of the field, -1 is blue side of the field
@Disabled
@Config
@Autonomous
public class OldSynchronousAuto extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    PivotingCamera camera;
    SignalPipeline signalPipeline = new SignalPipeline();
    AutoScoringMech scoringMech;

    int side = 1;
    String sidename;

    Pose2d startPos;
    Pose2d parkPos;
    Pose2d parkPos1;
    Pose2d parkPos2;
    Pose2d parkPos3;
    Pose2d preloadScoringPos;
    TrajectorySequence driveToPreloadPos;
    TrajectorySequence toStackFromPreload;
    double grabApproachVelo = 5;
    TrajectorySequence toJunction;
    TrajectorySequence park;

    @Override
    public void runOpMode(){
        // Init
        // Bind stuff to the hardwaremap
        drive = new SampleMecanumDrive(hardwareMap);
        scoringMech = new AutoScoringMech(hardwareMap);
        camera = new PivotingCamera(hardwareMap, signalPipeline);

        ElapsedTime pipelineThrottle = new ElapsedTime(100000000);

        // Init loop
        while (!isStarted()&&!isStopRequested()){
            if (gamepad1.circle) side = 1; // Red alliance
            if (gamepad1.cross) side = -1; // Blue alliance
            AutoToTele.allianceSide = side;
            // Is doing an if else statement on one line bad?
            if (side == 1) sidename = "Red Terminal"; else sidename = "Blue Terminal";

            // Recompute trajectories every second
            if (pipelineThrottle.milliseconds() > 1000){

                startPos = new Pose2d(-35.8, -63*side, Math.toRadians(-90*side));
                drive.setPoseEstimate(startPos);

                parkPos1 = new Pose2d(-58.5, -13*side, Math.toRadians(180*side));
                parkPos2 = new Pose2d(-36, -13*side, Math.toRadians(180*side));
                parkPos3 = new Pose2d(-11, -13*side, Math.toRadians(180*side));

                switch(signalPipeline.getParkPos()){
                    case 1:
                        // Switch 1 and 3 if we're on blue terminal
                        if (side == 1){
                            parkPos = parkPos1;
                        } else {
                            parkPos = parkPos3;
                        }
                        break;
                    case 2:
                        // We don't have to change the middle positon ever
                        parkPos = parkPos2;
                        break;
                    case 3:
                        // Switch 3 and 1 if we're on blue terminal
                        if (side == 1) {
                            parkPos = parkPos3;
                        } else {
                            parkPos = parkPos1;
                        }
                        break;
                }

                // Update trajectories
                preloadScoringPos = new Pose2d(-37, -7*side, Math.toRadians(131*side));

                driveToPreloadPos = drive.trajectorySequenceBuilder(startPos)
                        .lineToSplineHeading(preloadScoringPos)
                        .build();

                toStackFromPreload = drive.trajectorySequenceBuilder(driveToPreloadPos.end())
                        .setTangent(Math.toRadians(-120*side))
                        .splineToSplineHeading(new Pose2d(-55,-12.5*side, Math.toRadians(180*side)), Math.toRadians(180*side))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(grabApproachVelo, DriveConstants.MAX_ANG_VEL, 13.2))
                        .lineTo(new Vector2d(-64.5, -12.5*side))
                        .build();

                toJunction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(-30, -12*side))
                        .splineTo(new Vector2d(-12, -14*side), Math.toRadians(-30*side))
                        .build();

                park = drive.trajectorySequenceBuilder(toJunction.end())
                        .lineToSplineHeading(parkPos)
                        .build();

                // Display what park zone the robot is planning on going to and other things in telemetry
                telemetry.addLine(sidename);
                telemetry.addData("alliance", AutoToTele.allianceSide);
                telemetry.addData("park zone", signalPipeline.getParkPos());
                telemetry.addLine("CHECK THE AUTO, REMEMBER NANO FINALS 3!");
                telemetry.update();
                pipelineThrottle.reset();
            } // End of throttled section
        }

        waitForStart();
        camera.stopStreaming();
        // Start of actual auto instructions
        if (opModeIsActive()){
            // Auto code
            scoringMech.closeClaw(); // Grip the preload
            sleep((long) Arm.clawActuationTime);
            // Move the v4b vertical to save time when scoring and stop the cone from dragging on the ground
            scoringMech.preMoveV4b();
            // Drive off and score the preload
            drive.followTrajectorySequence(driveToPreloadPos);
            scoringMech.score(Lift.mediumPos);

            drive.followTrajectorySequenceAsync(toStackFromPreload);
            // While the claw sees no cone, drive to the stack
            while(!scoringMech.getConeStatus()){
                drive.update();
                scoringMech.grabOffStackAsync(4, scoringMech.getConeStatus());
            }
            // Cancel the toStack trajectory
            drive.breakFollowing();
            while(!(scoringMech.getStackGrabbingState() == AutoScoringMech.StackGrabbingState.DONE)){
                scoringMech.grabOffStackAsync(4, scoringMech.getConeStatus());
            }
            // Update the toJunction trajectory
            toJunction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(-30, -12*side))
                    .splineToSplineHeading(new Pose2d(-21, -14*side, Math.toRadians(150*side)), Math.toRadians(-30*side))
                    .build();

            // Score that cone we picked up
            drive.followTrajectorySequence(toJunction);
            scoringMech.resetScoringState();
            scoringMech.score(Lift.highPos);

            drive.followTrajectorySequence(park); // Big 20 points

            // Save this stuff at the end to calibrate feild centric automatically
            AutoToTele.endOfAutoPose = drive.getPoseEstimate();
            AutoToTele.endOfAutoHeading = drive.getExternalHeading();
        }
    }
}

//luke was here
// And Ely