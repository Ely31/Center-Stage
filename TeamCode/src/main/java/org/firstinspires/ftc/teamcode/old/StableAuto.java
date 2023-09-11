package org.firstinspires.ftc.teamcode.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.PivotingCamera;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.Utility;
import org.firstinspires.ftc.teamcode.vision.workspace.SignalPipeline;

//this autonomous is meant for if you start on the left side of the field
//regular is the red side of the field, -1 is blue side of the field
@Disabled
@Config
@Autonomous
public class StableAuto extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    PivotingCamera camera;
    SignalPipeline signalPipeline = new SignalPipeline();
    Arm arm;
    Lift lift;

    int side = 1;
    String sidename;

    Pose2d startPos;
    Pose2d parkPos;
    Pose2d parkPos1;
    Pose2d parkPos2;
    Pose2d parkPos3;
    public static double scoringPosX = -10;
    public static double scoringPosY = -32.5;
    public static Pose2d preloadScoringPos = new Pose2d(scoringPosX, scoringPosY, Math.toRadians(-135));
    TrajectorySequence driveToScoringPos;
    TrajectorySequence park;

    enum ScoringState{
        EXTENDING,
        WAIT,
        WAIT2,
        WAIT3,
        RETRACTING,
        DONE
    }
    ScoringState currentScoringState = ScoringState.EXTENDING;

    @Override
    public void runOpMode(){
        // Init
        // Bind stuff to the hardwaremap
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        lift = new Lift(hardwareMap);
        camera = new PivotingCamera(hardwareMap, signalPipeline);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        ElapsedTime pipelineThrottle = new ElapsedTime();
        ElapsedTime scoringWait = new ElapsedTime();

        // Init loop
        while (!isStarted()&&!isStopRequested()){
            if (gamepad1.circle) side = 1; // Red alliance
            if (gamepad1.cross) side = -1; // Blue alliance
            AutoToTele.allianceSide = side;
            // Is doing an if else statement on one line bad?
            if (side == 1) sidename = "Red Terminal"; else sidename = "Blue Terminal";

            // Recompute trajectories every second
            if (pipelineThrottle.milliseconds() > 1000){

                startPos = new Pose2d(-35.8, -60.6*side, Math.toRadians(-90*side));
                drive.setPoseEstimate(startPos);

                parkPos1 = new Pose2d(-58.5,    -34*side, Math.toRadians(-90*side));
                parkPos2 = new Pose2d(-36,      -34*side, Math.toRadians(-90*side));
                parkPos3 = new Pose2d(-11,      -34*side, Math.toRadians(-90*side));

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
                preloadScoringPos = new Pose2d(-13, -43*side, Math.toRadians(-127*side));

                driveToScoringPos = drive.trajectorySequenceBuilder(startPos)
                        .back(2.5)
                        .lineToSplineHeading(new Pose2d(-12.5, -56*side,Math.toRadians(-90*side)))
                        .lineToSplineHeading(preloadScoringPos)
                        .build();

                park = drive.trajectorySequenceBuilder(driveToScoringPos.end())
                        .lineToSplineHeading(new Pose2d(-13, -36, Math.toRadians(-90)))
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
            arm.closeClaw();
            sleep(500);
            drive.followTrajectorySequence(driveToScoringPos);

            // While it isn't finished scoring, run an FSM
            while (!(currentScoringState == ScoringState.DONE)){
                switch (currentScoringState){
                    case EXTENDING:
                        lift.goToHigh();
                        // Move on if the lift is all the way up
                        if (Utility.withinErrorOfValue(lift.getHeight(), Lift.highPos, 0.5)) {
                            arm.scorePassthrough(); // Move the v4b over the junction
                            scoringWait.reset();
                            currentScoringState = ScoringState.WAIT;
                        }
                        break;
                    case WAIT:
                        if (scoringWait.seconds() > 1.5){ // Wait for the v4b to move all the way
                            arm.openClaw(); // Drop the cone
                            scoringWait.reset();
                            currentScoringState = ScoringState.WAIT2;
                        }
                        break;
                    case WAIT2:
                        if (scoringWait.seconds() > 0.5){ // Wait for the cone to drop
                            arm.grabPassthrough(); // Move the v4b inside the bot
                            scoringWait.reset();
                            currentScoringState = ScoringState.WAIT3;
                        }
                        break;
                    case WAIT3:
                        if (scoringWait.seconds() > 0.5){ // Wait for the v4b to retract all the way
                            currentScoringState = ScoringState.RETRACTING;
                        }
                        break;
                    case RETRACTING:
                        lift.retract(); // Bring the lift down
                        // Move on if the lift is all the way down
                        if (Utility.withinErrorOfValue(lift.getHeight(), Lift.retractedPos, 0.5)) {
                            currentScoringState = ScoringState.DONE; // Finish
                        }
                        break;
                }
                // Always update the lift, no matter what state of scoring it's in
                lift.update();
            }

            drive.followTrajectorySequence(park);

            // Save this stuff at the end to calibrate feild centric automatically
            AutoToTele.endOfAutoPose = drive.getPoseEstimate();
            AutoToTele.endOfAutoHeading = drive.getExternalHeading();
        }
    }
}

//luke was here
// And Ely