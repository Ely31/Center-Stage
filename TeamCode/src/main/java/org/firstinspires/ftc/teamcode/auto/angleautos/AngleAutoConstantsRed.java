package org.firstinspires.ftc.teamcode.auto.angleautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoToTele;

import java.util.Random;

//max velo of our auto is 105 Nope luke that's wayyyy wrong
public class AngleAutoConstantsRed {
    SampleMecanumDrive drive;
    int randomMessageIndex;
    // Constructor
    public AngleAutoConstantsRed(SampleMecanumDrive drive){
        this.drive = drive;
        randomMessageIndex = new Random().nextInt(messageList.length);
    }

    //this modifies the routes of our auto code so that it will go on the opposite routes
    //needed for flexibility
    public boolean OppositeAuto = false;
    public boolean getOppositeAuto() {return OppositeAuto;}
    public void setOppositeAuto(boolean OppositeAuto) {this.OppositeAuto = OppositeAuto;}

    //allows us to drop white pixels in the backstage to avoid backdrop collisions in auto
    public boolean whitePixelDropBackstage1 = false;
    public boolean getWhitePixelDropBackstage1() {return whitePixelDropBackstage1;}
    public void setWhitePixelDropBackstage1(boolean whitePixelDropBackstage1) {this.whitePixelDropBackstage1 = whitePixelDropBackstage1;}

    public boolean whitePixelDropBackstage2 = false;
    public boolean getWhitePixelDropBackstage2() {return whitePixelDropBackstage2;}
    public void setWhitePixelDropBackstage2(boolean whitePixelDropBackstage2) {this.whitePixelDropBackstage2 = whitePixelDropBackstage2;}

    //wingside auto modifier
    boolean wingSide = false;
    public boolean isWingSide() {
        return wingSide;
    }
    public void setWingSide(boolean wingSide) {
        this.wingSide = wingSide;
    }

    boolean parkingClose = true;
    public boolean isParkingClose() {
        return parkingClose;
    }
    public void setParkingClose(boolean parkingClose) {
        this.parkingClose = parkingClose;
    }

    //tape measure park modifier
    boolean tapeMeasurePark = false;
    public boolean isTapeMeasurePark(){ return tapeMeasurePark; }
    public void setTapeMeasurePark(boolean tapeMeasureParkA){this.tapeMeasurePark = tapeMeasureParkA;}

    public int correctedSpikeMarkPos = 1;
    public int getCorrectedSpikeMarkPos() {
        return correctedSpikeMarkPos;
    }

    // This is used so we don't put our pixel in the same slot as our partner. Hopefully. We'll see.
    double dropOffset = 0;
    boolean dropIsOffset = false;
    public void setDropIsOffset(boolean value){
        dropIsOffset = value;
    }
    public boolean isDropOffset(){
        return dropIsOffset;
    }

    public int numCycles = 2;
    public int getNumCycles() {return numCycles;}
    public void setNumCycles(int numCycles) {this.numCycles = numCycles;}

    public int numFinishedCycles = 0;
    public void addFinishedCycle(){
        numFinishedCycles ++;
    }
    public int getNumFinishedCycles(){
        return numFinishedCycles;
    }

    public int delaySeconds = 0; // To be used to avoid collisions
    public int getDelaySeconds(){return delaySeconds;}
    public void setDelaySeconds(int seconds){
        delaySeconds = seconds;
    }

    public boolean avoidYellows = true;
    public void setAvoidYellows(boolean val){
        avoidYellows = val;
    }
    public boolean isAvoidingYellows(){
        return avoidYellows;
    }

    private int alliance = 1;
    public int getAlliance() {return alliance;}
    public void setAlliance(int alliance) {this.alliance = alliance;}
    // 1 is red, -1 is blue

    public void updateCorrectedSpikeMarkPos(int visionResult){
        switch(visionResult){
            case 1:
                correctedSpikeMarkPos = 1;
                break;
            case 2:
                correctedSpikeMarkPos = 2;
                break;
            case 3:
                correctedSpikeMarkPos = 3;
                break;
        }
    }

    Pose2d startPos;
    public TrajectorySequence dropOffPurplePixel;
    public TrajectorySequence scoreYellowPixel;
    public TrajectorySequence toStack;
    public TrajectorySequence toStackWPB;
    public TrajectorySequence scoreWhitePixels;
    public TrajectorySequence scoreWhitePixelsBackstage;
    public TrajectorySequence park;

    enum ParkingStuff{
        PARK_CLOSE,
        PARK_FAR,
        TAPE_MEASURE_PARK
    }

    public void updateTrajectories() {
        // Change start pose, pretty important
        if (isWingSide()) {
            startPos = new Pose2d(-35.25, -63.5, Math.toRadians(-90));
        } else {
            startPos = new Pose2d(11.75, -63.5, Math.toRadians(-90));
        }
        // Switch this so we don't drop our pixel in the same spot as our partner
        dropOffset = (isDropOffset() ? 2.5 : 0);

        // Ahhhh we have to have six unique purple pixel trajectories
        double yellowPixelYCoord = -27.5;
        final double baseYellowPixelYCoord = -29;
        final double yellowPixelXCoord = 52;
        final double whitePixelXCoord = 50.4;
        double whitePixelYCoord = -29.5;
        final double wingSideWhiteY = -26;
        final double boardSideWhiteY = -42.5;

        //wing side auto
        if (isWingSide()) {
            double afterPurpleTangent = 180;
            // Avoid dropping whites on top of a future mosaic if we want
            if(getOppositeAuto()){
                if(avoidYellows && correctedSpikeMarkPos == 3) {whitePixelYCoord = wingSideWhiteY;}
                whitePixelYCoord = boardSideWhiteY + 5;
            }
            else{
                if (avoidYellows && correctedSpikeMarkPos == 1) {whitePixelYCoord = boardSideWhiteY;}
                whitePixelYCoord = wingSideWhiteY;
            }

            switch (correctedSpikeMarkPos) {
                case 1:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                            .lineToSplineHeading(new Pose2d(-46, -20.5, Math.toRadians(90)))
                            .build();
                    yellowPixelYCoord = baseYellowPixelYCoord - 1.5 - dropOffset;
                    afterPurpleTangent = 90;
                    break;
                case 2:
                    yellowPixelYCoord = baseYellowPixelYCoord - 7.2 - dropOffset;
                    afterPurpleTangent = 90;

                    if (getOppositeAuto()) {
                        dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                                .lineToSplineHeading(new Pose2d(-41, -36, Math.toRadians(-90)))
                                .build();
                    } else {
                        dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                                .lineToSplineHeading(new Pose2d(-40, -11.5, Math.toRadians(89.9))) // 89.9 so it turns CW
                                .build();
                    }
                    break;
                default:
                    //TODO: Fix the error with 3rd position where it overshoots
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(-37, -34, Math.toRadians(179.5)))
                            .build();
                    yellowPixelYCoord = baseYellowPixelYCoord - 13.4 - dropOffset;
                    break;
            }

            if (getOppositeAuto()){
                scoreYellowPixel = drive.trajectorySequenceBuilder(dropOffPurplePixel.end())
                        //this path will drive through the truss instead of the truss door
                        .lineToSplineHeading(new Pose2d(-41, (-46.5), Math.toRadians(0)))
                        //drive through the truss
                        .splineToConstantHeading(new Vector2d(14, (-58)), 0 )
                        // To the board
                        .splineToSplineHeading(new Pose2d(yellowPixelXCoord + 0.6, yellowPixelYCoord, Math.toRadians(0)), Math.toRadians(0))
                        .build();

                toStack = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(28, -58), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-30, -57), Math.toRadians(180))
                        // Ok we're out of the truss now
                        .resetVelConstraint()
                        .splineToSplineHeading(new Pose2d((getNumFinishedCycles() == 0 ? -56 : -57), (-40.5), Math.toRadians(-20)), Math.toRadians(110))
                        .build();

                scoreWhitePixels = drive.trajectorySequenceBuilder(toStack.end())
                        .lineToSplineHeading(new Pose2d(-50, (-48), Math.toRadians(0)))
                        .splineToConstantHeading(new Vector2d(12, (-58)), 0)
                        .resetVelConstraint()
                        .splineTo(new Vector2d(whitePixelXCoord, (whitePixelYCoord)), 0)
                        .build();
            }
            else {
                scoreYellowPixel = drive.trajectorySequenceBuilder(dropOffPurplePixel.end())
                        //This path will run the normal wingside path (go through the truss door)
                        .setTangent(Math.toRadians(afterPurpleTangent))
                        .splineToSplineHeading(new Pose2d(-29, -12, Math.toRadians(0)), Math.toRadians(0))
                        // Drive under the door
                        .resetVelConstraint()
                        .splineTo(new Vector2d(21.5, -12), Math.toRadians(0))
                        // To the board
                        .splineToConstantHeading(new Vector2d(yellowPixelXCoord - 1, yellowPixelYCoord), Math.toRadians(0))
                        .build();

                toStack = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(28, -3), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-30, -3), Math.toRadians(180))
                        // Ok we're out of the truss now
                        .resetVelConstraint()
                        .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(180 ))
                        .build();

                scoreWhitePixels = drive.trajectorySequenceBuilder(toStack.end())
                        .splineToConstantHeading(new Vector2d(24, -11), 0)
                        .resetVelConstraint()
                        .splineToConstantHeading(new Vector2d(whitePixelXCoord, whitePixelYCoord), 0)
                        .build();
            }
            // END OF WINGSIDE
        } else {
            // Board side
            // Avoid dropping whites on top of a future mosaic if we want
            // additional logic for opposite side autos
            if(getOppositeAuto()){
                if(avoidYellows && correctedSpikeMarkPos == 1) whitePixelYCoord = boardSideWhiteY;
                else whitePixelYCoord = wingSideWhiteY;
            }
            else{
                if (avoidYellows && correctedSpikeMarkPos == 3){
                    whitePixelYCoord = wingSideWhiteY;
                }
                else whitePixelYCoord = boardSideWhiteY;
            }

            switch (correctedSpikeMarkPos) {
                case 1:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(50))
                            .lineToSplineHeading(new Pose2d(13, -34, Math.toRadians(0)))
                            .resetAccelConstraint()
                            .build();
                    yellowPixelYCoord = baseYellowPixelYCoord - 1.5 - dropOffset;
                    break;
                case 2:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(20))
                            .lineToSplineHeading(new Pose2d(28.8, -24.5, Math.toRadians(0)))
                            .resetAccelConstraint()
                            .build();
                    yellowPixelYCoord = baseYellowPixelYCoord - 7.5 - dropOffset;
                    break;
                default:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(35, -34, Math.toRadians(0)))
                            .build();
                    yellowPixelYCoord = baseYellowPixelYCoord - 13.4 - dropOffset;
                    break;
            }

            scoreYellowPixel = drive.trajectorySequenceBuilder(dropOffPurplePixel.end())
                    // Go straight to the board
                    .lineToSplineHeading(new Pose2d(yellowPixelXCoord, yellowPixelYCoord, Math.toRadians(0)))
                    .build();

            if (getOppositeAuto()) {
                //this path runs through the truss door on board side
                toStack = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(28, -10), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-30, -10), Math.toRadians(180))
                        // Ok we're out of the truss now
                        .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(180))
                        .resetVelConstraint()
                        .build();

                scoreWhitePixels = drive.trajectorySequenceBuilder(toStack.end())
                        .splineToConstantHeading(new Vector2d(24, -11), 0)
                        .resetVelConstraint()
                        .resetAccelConstraint()
                        .splineTo(new Vector2d(whitePixelXCoord + 0.6, whitePixelYCoord), 0)
                        .build();
            }
            else {
                //this path runs through the truss on board side
                toStack = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(28, -58.5), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-30, -57.5), Math.toRadians(180))
                        .resetVelConstraint()
                        // Ok we're out of the truss now
                        .splineToSplineHeading(new Pose2d((getNumFinishedCycles() == 0 ? -55 : -55), (-40), Math.toRadians(-20)), Math.toRadians(110))
                        .build();

                scoreWhitePixels = drive.trajectorySequenceBuilder(toStack.end())
                        .lineToSplineHeading(new Pose2d(-50, (-46), Math.toRadians(0)))
                        .splineToConstantHeading(new Vector2d(14, (-59)), 0)
                        .resetVelConstraint()
                        .splineToConstantHeading(new Vector2d(whitePixelXCoord, (whitePixelYCoord)), 0)
                        .build();
            }
            // END OF BOARDSIDE
        }

        //All of the stuff below is Wingside/Audience side stuff
        ParkingStuff parkingStuff = null;
        if (tapeMeasurePark) { parkingStuff = ParkingStuff.TAPE_MEASURE_PARK;}
        else if(parkingClose && !tapeMeasurePark) { parkingStuff = ParkingStuff.PARK_CLOSE;}
        else if(!parkingClose && !tapeMeasurePark) {parkingStuff = ParkingStuff.PARK_FAR;}

        switch (parkingStuff) {
            case PARK_CLOSE:
                park = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(50, -59.5, Math.toRadians(0)), Math.toRadians(-90))
                        .build();

                if(getOppositeAuto()){
                    scoreWhitePixelsBackstage = drive.trajectorySequenceBuilder(toStack.end())
                            .splineToConstantHeading(new Vector2d(24, -11), 0)
                            .resetVelConstraint()
                            .resetAccelConstraint()
                            .splineToConstantHeading(new Vector2d(45, -12), Math.toRadians(90))
                            .build();

                    toStackWPB = drive.trajectorySequenceBuilder(scoreWhitePixelsBackstage.end())
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(28, -10), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-30, -10), Math.toRadians(180))
                            // Ok we're out of the truss now
                            .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(180))
                            .resetVelConstraint()
                            .build();
                }
                else{
                    scoreWhitePixelsBackstage = drive.trajectorySequenceBuilder(toStack.end())
                            .lineToSplineHeading(new Pose2d(-50, (-48), Math.toRadians(0)))
                            .splineToConstantHeading(new Vector2d(45, (-59.5)), 0)
                            .resetVelConstraint()
                            .build();

                    toStackWPB = drive.trajectorySequenceBuilder(scoreWhitePixelsBackstage.end())
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(28, -58), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-30, -57), Math.toRadians(180))
                            .resetVelConstraint()
                            // Ok we're out of the truss now
                            .splineToSplineHeading(new Pose2d((getNumFinishedCycles() == 0 ? -56 : -57 ), (-39.5), Math.toRadians(-20)), Math.toRadians(110))
                            .build();
                }
                break;

            case PARK_FAR:
                park = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(50, -12, Math.toRadians(0)), Math.toRadians(90))
                        .build();

                if(getOppositeAuto()){
                    scoreWhitePixelsBackstage = drive.trajectorySequenceBuilder(toStack.end())
                            .lineToSplineHeading(new Pose2d(-50, (-48), Math.toRadians(0)))
                            .splineToConstantHeading(new Vector2d(12, (-58)), 0)
                            .resetVelConstraint()
                            .splineToConstantHeading(new Vector2d(45, -61), Math.toRadians(-90))
                            .build();

                    toStackWPB = drive.trajectorySequenceBuilder(scoreWhitePixelsBackstage.end())
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(28, -58), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-30, -57), Math.toRadians(180))
                            // Ok we're out of the truss now
                            .resetVelConstraint()
                            .splineToSplineHeading(new Pose2d((getNumFinishedCycles() == 0 ? -56 : -57), (-40.5), Math.toRadians(-20)), Math.toRadians(110))
                            .build();
                }
                else{
                    scoreWhitePixelsBackstage = drive.trajectorySequenceBuilder(toStack.end())
                            .splineToConstantHeading(new Vector2d(24, -11), 0)
                            .resetVelConstraint()
                            .resetAccelConstraint()
                            .splineToConstantHeading(new Vector2d(45, -12), Math.toRadians(90))
                            .build();

                    toStackWPB = drive.trajectorySequenceBuilder(scoreWhitePixelsBackstage.end())
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(28, -9.5), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-30, -9.5), Math.toRadians(180))
                            .resetVelConstraint()
                            .resetAccelConstraint()
                            .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(180))
                            .build();
                }
                break;

            case TAPE_MEASURE_PARK:
                park = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(28, -10), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-30, -12), Math.toRadians(180))
                        .build();

                scoreWhitePixelsBackstage = drive.trajectorySequenceBuilder(toStack.end())
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(45, -12, Math.toRadians(0)), Math.toRadians(90))
                        .build();
                break;
        }
    } // End of updateTrajectories

    public void saveAutoPose(){
        AutoToTele.endOfAutoPose = drive.getPoseEstimate();
        AutoToTele.endOfAutoHeading = drive.getPoseEstimate().getHeading();
    }

    // Telemetry stuff
    public void addTelemetry(Telemetry telemetry){
        // Write the alliance in its color
        telemetry.addData("Side", (isWingSide() ? "Wing /\\" : "Board [")); // First time using this funny switchy thing
        telemetry.addData("Corrected Spike Mark Pos", getCorrectedSpikeMarkPos());
        telemetry.addData("Delay in seconds", getDelaySeconds());
        telemetry.addData("Number of cycles", getNumCycles());
        telemetry.addData("Parking close", isParkingClose());
        telemetry.addData("Drop is offset", isDropOffset());
        telemetry.addData("Avoiding yellows", isAvoidingYellows());
        telemetry.addData("Tape measure park", isTapeMeasurePark());
        telemetry.addData("Opposite autos", getOppositeAuto());
        telemetry.addData("White pixels backstage first cycle (1)", getWhitePixelDropBackstage1());
        telemetry.addData("White pixel backstage second cycle (2)", getWhitePixelDropBackstage2());
        telemetry.addLine();
        telemetry.addLine(autoConfigToEnglish());
        telemetry.addLine();
        telemetry.addLine(ramdomAutoCheckMessage());

        prevConfigToEnglish = autoConfigToEnglish();
    }
    String[] messageList = {
            "CHECK THE AUTO, REMEMBER NANO FINALS 3!",
            "Ok apparently it was Nano Finals 2",
            "Run the right auto kids!",
            "Is it red? is it blue?",
            "Is it left? is it right?",
            "Are you SURE this is the program you want?",
            "¡Ejecute el auto correcto!",
            "올바른 자동 실행",
            "Oi mate, didjya checkit eh?",
            "What do those numbers say, hmmmm? Hmmmm?",
            "C'mon man, just take a second to read the stuff",
            "运行正确的自动",
            "Don't waste the potential of this bot",
            "How many cycles are we doin'?",
            "Where are we parkin'?",
            "Look. At. The. Side.",
            "Look at the bot, now look at the screen",
            "(insert mildly funny comment about auto)",
            "ELYYYYY...",
            "LUUUKEE...",
            ":)",
            "Don't lose worlds!",
            "Pay attention bro",
            "Don't pull a brainstormers FF finals",
            "Guys this auto's more complicated than last year",
            "Is it wing? is it board?",
            "I am watching you"
    };
    String ramdomAutoCheckMessage(){
        //look up the index of the randomly generated number in the array of messages and return that message
        return messageList[randomMessageIndex];
    }
    String autoConfigToEnglish(){
        String spikeMarkDescription;
        String yellowPixelDescription;
        switch (getCorrectedSpikeMarkPos()){
            case 1:
                spikeMarkDescription = "closest to the wing";
                yellowPixelDescription = "closest to the center of the field";
                break;
            case 2:
                spikeMarkDescription = "in the center";
                yellowPixelDescription = "in the center";
                break;
            default:
                spikeMarkDescription = "closest to the board";
                yellowPixelDescription = "closest to the wall";
                break;
        }
        return (
                        " You are on the side of the field closest to the " + (isWingSide() ? "wing" : "board") + "."
                        + " The bot will wait " + getDelaySeconds() + " Seconds before moving the purple pixel to the spike mark " + spikeMarkDescription
                        + " and scoring the yellow pixel " + yellowPixelDescription + "."
                        + " The yellow pixel will be placed in that slot closest to " + (isDropOffset() ? "you" : "the center of the field") + "."
                        + " It'll do " + getNumCycles() + " Cycles."
                        + " The bot will then park closest to " + (isParkingClose() ? "you" : "the center of the field") + "."
                        + "Opposite auto is" + (getOppositeAuto()) + "."
                        + "Tape measure park is" + (isTapeMeasurePark()) + "."
                        + "White pixels backstageA is" + (getWhitePixelDropBackstage1()) + "."
                        + "White pixels backstageB is" + (getWhitePixelDropBackstage2()) + "."
                        + "\n" + "Sound right?"
        );
    }
    String prevConfigToEnglish = "";
}