package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoToTele;

import java.util.Random;

//max velo of our auto is 105
public class ExtendoAutoConstants2 {
    SampleMecanumDrive drive;
    int randomMessageIndex;
    // Constructor
    public ExtendoAutoConstants2(SampleMecanumDrive drive){
        this.drive = drive;
        randomMessageIndex = new Random().nextInt(messageList.length);
    }

    // 1 is red, -1 is blue
    private int alliance = 1;
    public int getAlliance() {return alliance;}
    public void setAlliance(int alliance) {this.alliance = alliance;}
    public boolean isRedAlliance(){
        return alliance == 1;
    }
    public String allianceToString(){
        if (alliance == 1) return "red alliance";
        else return "blue alliance";
    }

    //this modifys the routes of our auto code so that it will go on the opposite routes
    //needed for flexibility
    private boolean OppositeAuto = false;
    public boolean getOppositeAuto() {return OppositeAuto;}
    public void setOppositeAuto(boolean OppositeAuto) {this.OppositeAuto = OppositeAuto;}

    //allows us to drop white pixels in the backstage to avoid backdrop collisions in auto
    private boolean whitePixelDropBackstage1 = false;
    public boolean getWhitePixelDropBackstage1() {return whitePixelDropBackstage1;}
    public void setWhitePixelDropBackstage1(boolean whitePixelDropBackstage1) {this.whitePixelDropBackstage1 = whitePixelDropBackstage1;}

    private boolean whitePixelDropBackstage2 = false;
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

    private int correctedSpikeMarkPos = 1;
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

    private int numCycles = 2;
    public int getNumCycles() {return numCycles;}
    public void setNumCycles(int numCycles) {this.numCycles = numCycles;}

    private int numFinishedCycles = 0;
    public void addFinishedCycle(){
        numFinishedCycles ++;
    }
    public int getNumFinishedCycles(){
        return numFinishedCycles;
    }

    private int delaySeconds = 0; // To be used to avoid collisions
    public int getDelaySeconds(){return delaySeconds;}
    public void setDelaySeconds(int seconds){
        delaySeconds = seconds;
    }

    private boolean avoidYellows = true;
    public void setAvoidYellows(boolean val){
        avoidYellows = val;
    }
    public boolean isAvoidingYellows(){
        return avoidYellows;
    }

    boolean sketchyBlueOffsets = true;
    double sketchyBlueStackOffsetY;
    double sketcyBlueStackOffsetX;
    double sketchyBlueBoardOffsetX;

    public void updateCorrectedSpikeMarkPos(int visionResult){
        switch(visionResult){
            case 1:
                // Switch 1 and 3 if we're on blue terminal
                if (getAlliance() == 1){
                    correctedSpikeMarkPos = 1;
                } else {
                    correctedSpikeMarkPos = 3;
                }
                break;
            case 2:
                // We don't have to change the middle positon however
                correctedSpikeMarkPos = 2;
                break;
            case 3:
                // Switch 3 and 1 if we're on blue terminal
                if (getAlliance() == 1) {
                    correctedSpikeMarkPos = 3;
                } else {
                    correctedSpikeMarkPos = 1;
                }
                break;
        }
    }

    Pose2d startPos;
    public TrajectorySequence dropOffPurplePixel;
    public TrajectorySequence scoreYellowPixel;
    public TrajectorySequence toStack;
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
            startPos = new Pose2d(-35.25, -63.5 * alliance, Math.toRadians(-90 * alliance));
        } else {
            startPos = new Pose2d(11.75, -63.5 * alliance, Math.toRadians(-90 * alliance));
        }
        // Switch this so we don't drop our pixel in the same spot as our partner
        dropOffset = (isDropOffset() ? 2.4 : 0);

        // Ahhhh we have to have six unique purple pixel trajectories
        double yellowPixelYCoord = -28;
        final double baseYellowPixelYCoord = -29;
        final double yellowPixelXCoord = 52;
        final double whitePixelXCoord = 51.3;
        double whitePixelYCoord = -29.5;
        final double wingSideWhiteY = -30;
        final double boardSideWhiteY = -40.5;

        //wing side auto
        if (isWingSide()) {

            if (!isRedAlliance() && sketchyBlueOffsets) {
                sketchyBlueStackOffsetY = 4;
                sketcyBlueStackOffsetX = 1.2;
                sketchyBlueBoardOffsetX = 1.1;
            } else {
                sketchyBlueStackOffsetY = 0;
                sketcyBlueStackOffsetX = 0;
                sketchyBlueBoardOffsetX = 0;
            }
            double afterPurpleTangent = 180;
            // Avoid dropping whites on top of a future mosaic if we want
            if(getOppositeAuto()){
                if(avoidYellows && correctedSpikeMarkPos == 3) whitePixelYCoord = wingSideWhiteY;
                whitePixelYCoord = boardSideWhiteY + 5;
            }
            else{
                if (avoidYellows && correctedSpikeMarkPos == 1) whitePixelYCoord = boardSideWhiteY;
                whitePixelYCoord = wingSideWhiteY;
            }

            switch (correctedSpikeMarkPos) {
                case 1:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(-46, -20.5 * alliance, Math.toRadians(90 * alliance)))
                            .build();
                    yellowPixelYCoord = baseYellowPixelYCoord - dropOffset;
                    afterPurpleTangent = 90;
                    break;
                case 2:
                    if (getOppositeAuto()) {
                        dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                                .lineToSplineHeading(new Pose2d(-41, -36 * alliance, Math.toRadians(-90 * alliance)))
                                .build();
                        yellowPixelYCoord = baseYellowPixelYCoord - 3 - dropOffset;
                        afterPurpleTangent = 90;
                    }
                    else {
                        dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                                .lineToSplineHeading(new Pose2d(-40, -11.5 * alliance, Math.toRadians(89.9 * alliance))) // 89.9 so it turns CW
                                .build();
                        yellowPixelYCoord = baseYellowPixelYCoord - 6 - dropOffset;
                        afterPurpleTangent = 90;
                    }
                    break;
                default:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(-37, -34 * alliance, Math.toRadians(179.5 * alliance)))
                            .build();
                    yellowPixelYCoord = baseYellowPixelYCoord - 9 - dropOffset;
                    break;
            }

            if (getOppositeAuto()){
                scoreYellowPixel = drive.trajectorySequenceBuilder(dropOffPurplePixel.end())
                        //this path will drive through the truss instead of the truss door
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(180), 14))
                        .lineToSplineHeading(new Pose2d(-41, (-46.5 + sketchyBlueStackOffsetY) * alliance, Math.toRadians(0 * alliance)))
                        //drive through the truss
                        .splineToConstantHeading(new Vector2d(12, (-58 + sketchyBlueStackOffsetY) * alliance), 0 * alliance)
                        // To the board
                        .resetVelConstraint()
                        .splineToSplineHeading(new Pose2d(yellowPixelXCoord + 0.6, yellowPixelYCoord * alliance, Math.toRadians(0 * alliance)), Math.toRadians(0 * alliance))
                        .build();

                toStack = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180 * alliance))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(180), 14))
                        .splineToConstantHeading(new Vector2d(28, -58 * alliance), Math.toRadians(180 * alliance))
                        .splineToConstantHeading(new Vector2d(-30, -57 * alliance), Math.toRadians(180 * alliance))
                        // Ok we're out of the truss now
                        .resetVelConstraint()
                        .splineToSplineHeading(new Pose2d((getNumFinishedCycles() == 0 ? -56 + sketcyBlueStackOffsetX : -57 + sketcyBlueStackOffsetX), (-40.5 + sketchyBlueStackOffsetY) * alliance, Math.toRadians(-20 * alliance)), Math.toRadians(110 * alliance))
                        .build();

                scoreWhitePixels = drive.trajectorySequenceBuilder(toStack.end())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(180), 14))
                        .lineToSplineHeading(new Pose2d(-50, (-48 + sketchyBlueStackOffsetY) * alliance, Math.toRadians(0 * alliance)))
                        .splineToConstantHeading(new Vector2d(12, (-58 + sketchyBlueStackOffsetY) * alliance), 0 * alliance)
                        .resetVelConstraint()
                        .splineTo(new Vector2d(whitePixelXCoord + sketchyBlueBoardOffsetX, (whitePixelYCoord) * alliance), 0 * alliance)
                        .build();
            }
            else {
                scoreYellowPixel = drive.trajectorySequenceBuilder(dropOffPurplePixel.end())
                        //This path will run the normal wingside path (go through the truss door)
                        .setTangent(Math.toRadians(afterPurpleTangent * alliance))
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(60))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80, Math.toRadians(180), 14))
                        .splineToSplineHeading(new Pose2d(-29, -12 * alliance, Math.toRadians(0 * alliance)), Math.toRadians(0 * alliance))
                        // Drive under the door
                        .splineTo(new Vector2d(20, -12 * alliance), Math.toRadians(0 * alliance))
                        .resetVelConstraint()
                        .resetAccelConstraint()
                        // To the board
                        .splineToSplineHeading(new Pose2d(yellowPixelXCoord + 0.6, yellowPixelYCoord * alliance, Math.toRadians(0 * alliance)), Math.toRadians(0 * alliance))
                        .build();

                toStack = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180 * alliance))
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(60))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80, Math.toRadians(180), 14))
                        .splineToConstantHeading(new Vector2d(28, -11 * alliance), Math.toRadians(180 * alliance))
                        .splineToConstantHeading(new Vector2d(-30, -11 * alliance), Math.toRadians(180 * alliance))
                        // Ok we're out of the truss now
                        .resetVelConstraint()
                        .resetAccelConstraint()
                        .splineToConstantHeading(new Vector2d(-55, -11 * alliance), Math.toRadians(180 * alliance))
                        .build();

                scoreWhitePixels = drive.trajectorySequenceBuilder(toStack.end())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80, Math.toRadians(180), 14))
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(60))
                        //.lineToSplineHeading(new Pose2d(-40, -11*alliance, Math.toRadians(0*alliance)))
                        .splineToConstantHeading(new Vector2d(24, -11 * alliance), 0 * alliance)
                        .resetVelConstraint()
                        .resetAccelConstraint()
                        .splineTo(new Vector2d(whitePixelXCoord + 0.6, whitePixelYCoord * alliance), 0 * alliance)
                        .build();
            }
            // END OF WINGSIDE
        } else {
            if (!isRedAlliance() && sketchyBlueOffsets) {
                sketchyBlueStackOffsetY = 4;
                sketcyBlueStackOffsetX = 1.2;
                sketchyBlueBoardOffsetX = 1.1;
            } else {
                sketchyBlueStackOffsetY = 0;
                sketcyBlueStackOffsetX = 0;
                sketchyBlueBoardOffsetX = 0;
            }
            // Board side
            // Avoid dropping whites on top of a future mosaic if we want
            //additional logic for opposite side autos
            if(getOppositeAuto()){
                if(avoidYellows && correctedSpikeMarkPos == 1) whitePixelYCoord = boardSideWhiteY;
                else whitePixelYCoord = wingSideWhiteY;
            }
            else{
                if (avoidYellows && correctedSpikeMarkPos == 3) whitePixelYCoord = wingSideWhiteY;
                else whitePixelYCoord = boardSideWhiteY;
            }

            switch (correctedSpikeMarkPos) {
                case 1:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(13, -34 * alliance, Math.toRadians(0 * alliance)))
                            .build();
                    yellowPixelYCoord = baseYellowPixelYCoord - dropOffset;
                    break;
                case 2:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(28.8, -24.5 * alliance, Math.toRadians(0 * alliance)))
                            .build();
                    yellowPixelYCoord = baseYellowPixelYCoord - 5.5 - dropOffset;
                    break;
                default:
                    dropOffPurplePixel = drive.trajectorySequenceBuilder(startPos)
                            .lineToSplineHeading(new Pose2d(35, -34 * alliance, Math.toRadians(0 * alliance)))
                            .build();
                    yellowPixelYCoord = baseYellowPixelYCoord - 11.4 - dropOffset;
                    break;
            }

            scoreYellowPixel = drive.trajectorySequenceBuilder(dropOffPurplePixel.end())
                    // Go straight to the board
                    .lineToSplineHeading(new Pose2d(yellowPixelXCoord, yellowPixelYCoord * alliance, Math.toRadians(0 * alliance)))
                    .build();

            if (getOppositeAuto()) {
                //this path runs through the truss door on board side
                toStack = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180 * alliance))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.toRadians(180), 14))
                        .splineToConstantHeading(new Vector2d(28, -10 * alliance), Math.toRadians(180 * alliance))
                        .splineToConstantHeading(new Vector2d(-30, -10 * alliance), Math.toRadians(180 * alliance))
                        // Ok we're out of the truss now
                        .splineToConstantHeading(new Vector2d(-55, -11 * alliance), Math.toRadians(180 * alliance))
                        .resetVelConstraint()
                        .build();

                scoreWhitePixels = drive.trajectorySequenceBuilder(toStack.end())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.toRadians(180), 14))
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(60))
                        .splineToConstantHeading(new Vector2d(24, -11 * alliance), 0 * alliance)
                        .resetVelConstraint()
                        .resetAccelConstraint()
                        .splineTo(new Vector2d(whitePixelXCoord + 0.6, whitePixelYCoord * alliance), 0 * alliance)
                        .build();
            }
            else {
                //this path runs through the truss on board side
                toStack = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180 * alliance))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.toRadians(180), 14))
                        .splineToConstantHeading(new Vector2d(28, -57 * alliance), Math.toRadians(180 * alliance))
                        .splineToConstantHeading(new Vector2d(-30, -56 * alliance), Math.toRadians(180 * alliance))
                        // Ok we're out of the truss now
                        .resetVelConstraint()
                        .splineToSplineHeading(new Pose2d((getNumFinishedCycles() == 0 ? -56 + sketcyBlueStackOffsetX : -57 + sketcyBlueStackOffsetX), (-40.5 + sketchyBlueStackOffsetY) * alliance, Math.toRadians(-20 * alliance)), Math.toRadians(110 * alliance))
                        .build();

                scoreWhitePixels = drive.trajectorySequenceBuilder(toStack.end())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.toRadians(180), 14))
                        .lineToSplineHeading(new Pose2d(-50, (-48 + sketchyBlueStackOffsetY) * alliance, Math.toRadians(0 * alliance)))
                        .splineToConstantHeading(new Vector2d(12, (-59.5 + sketchyBlueStackOffsetY) * alliance), 0 * alliance)
                        .resetVelConstraint()
                        .splineTo(new Vector2d(whitePixelXCoord + sketchyBlueBoardOffsetX, (whitePixelYCoord) * alliance), 0 * alliance)
                        .build();
            }
            // END OF BOARDSIDE
        }

        ParkingStuff parkingStuff = null;
        if (tapeMeasurePark) { parkingStuff = ParkingStuff.TAPE_MEASURE_PARK;}
        else if(parkingClose && tapeMeasurePark == false) { parkingStuff = ParkingStuff.PARK_CLOSE;}
        else if(parkingClose == false && tapeMeasurePark == false) {parkingStuff = ParkingStuff.PARK_FAR;}

        switch (parkingStuff) {
            case PARK_CLOSE:
                park = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180 * alliance))
                        .splineToLinearHeading(new Pose2d(50, -61 * alliance, Math.toRadians(0 * alliance)), Math.toRadians(-90 * alliance))
                        .build();

                scoreWhitePixelsBackstage = drive.trajectorySequenceBuilder(toStack.end())
                        .setTangent(Math.toRadians(180 * alliance))
                        .splineToLinearHeading(new Pose2d(45, -61 * alliance, Math.toRadians(0 * alliance)), Math.toRadians(-90 * alliance))
                        .build();
                break;

            case PARK_FAR:
                park = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180 * alliance))
                        .splineToLinearHeading(new Pose2d(50, -12 * alliance, Math.toRadians(0 * alliance)), Math.toRadians(90 * alliance))
                        .build();

                scoreWhitePixelsBackstage = drive.trajectorySequenceBuilder(toStack.end())
                        .setTangent(Math.toRadians(180 * alliance))
                        .splineToLinearHeading(new Pose2d(45, -12 * alliance, Math.toRadians(0 * alliance)), Math.toRadians(90 * alliance))
                        .build();
                break;

            case TAPE_MEASURE_PARK:
                park = drive.trajectorySequenceBuilder(getNumFinishedCycles() == 0 ? scoreYellowPixel.end() : scoreWhitePixels.end())
                        .setTangent(Math.toRadians(180 * alliance))
                        .splineToLinearHeading(new Pose2d(-10, -12 * alliance, Math.toRadians(0 * alliance)), Math.toRadians(90 * alliance))
                        .build();

                scoreWhitePixelsBackstage = drive.trajectorySequenceBuilder(toStack.end())
                        .setTangent(Math.toRadians(180 * alliance))
                        .splineToLinearHeading(new Pose2d(45, -12 * alliance, Math.toRadians(0 * alliance)), Math.toRadians(90 * alliance))
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
        telemetry.addLine("<font color =#"+ (isRedAlliance() ? "ff0000>" : "0099ff>") + allianceToString() + "</font>" );
        telemetry.addData("Side", (isWingSide() ? "Wing /\\" : "Board [")); // First time using this funny switchy thing
        telemetry.addData("Corrected Spike Mark Pos", getCorrectedSpikeMarkPos());
        telemetry.addData("Delay in seconds", getDelaySeconds());
        telemetry.addData("Number of cycles", getNumCycles());
        telemetry.addData("Parking close", isParkingClose());
        telemetry.addData("Drop is offset", isDropOffset());
        telemetry.addData("Avoiding yellows", isAvoidingYellows());
        telemetry.addData("Tape measure park", isTapeMeasurePark());
        telemetry.addData("Opposite autos", getOppositeAuto());
        telemetry.addData("White pixels backstage last cycle", getWhitePixelDropBackstage1());
        telemetry.addData("White pixel backstage first cycle", getWhitePixelDropBackstage2());
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
                "You are on the " + allianceToString() + "."
                + " You are on the side of the field closest to the " + (isWingSide() ? "wing" : "board") + "."
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
