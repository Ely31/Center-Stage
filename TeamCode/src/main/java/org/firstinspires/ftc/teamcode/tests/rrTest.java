package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous(group = "test")
public class rrTest extends LinearOpMode {
    SampleMecanumDrive drive;
    TrajectorySequence test;

    final double distanceToWall = 142.0/2;
    final double distanceToBotEdgeLong = 9;
    final double distanceToBotEdgeShort = 9;

    Pose2d startpos = new Pose2d(-35,-(distanceToWall- distanceToBotEdgeLong),Math.toRadians(0));

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startpos);

        test =  drive.trajectorySequenceBuilder(startpos)
                .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(0)))
                .build();

        waitForStart();
        if (opModeIsActive()) {
            drive.followTrajectorySequence(test);
        }
    }
}
