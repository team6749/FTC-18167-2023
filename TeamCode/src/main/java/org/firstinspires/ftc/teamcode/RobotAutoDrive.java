/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

public abstract class RobotAutoDrive extends LinearOpMode {
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 4.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 1.0;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 1.0;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 1.0;   //  Clip the turn speed to this max value (adjust for your robot)

    private static int iter = 0;
//    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
//              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private boolean targetFound = false;
    RobotHardware robot = new RobotHardware(this);

    public void initBase() throws InterruptedException {
        robot.init();
//        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
//        initAprilTag();
        robot.initCameraTfodAndAprilTags();

//        if (USE_WEBCAM)
//            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

    }


    public int testSpikePos() throws InterruptedException{

        telemetry.addData("Start Detection","");
        telemetry.update();
        List<Recognition> currentRecognitions = robot.tfod.getFreshRecognitions();
        if (currentRecognitions != null) {
            telemetry.addData("# Objects Detected", currentRecognitions.size());
            telemetry.update();


        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            telemetry.addData("- Left/Right", "%.0f x %.0f", recognition.getLeft(), recognition.getRight());
            telemetry.update();

        }   // end for() loop
        }
        sleep(50);

        return -1;
    }

    public int determineSpikePos() throws InterruptedException {
        int spikeMark = 1;
        robot.turn(MAX_AUTO_TURN, 20, 3);
        List<Recognition> currentRecognitions = getRecognitions(null);
        currentRecognitions = getRecognitions(currentRecognitions);
        currentRecognitions = getRecognitions(currentRecognitions);

        if (currentRecognitions != null && currentRecognitions.size() > 0) {
            spikeMark = 2;
//            // Step through the list of recognitions and display info for each one.
//            for (Recognition recognition : currentRecognitions) {
//                if ("Pixel".equals(recognition.getLabel())) {
//                    double x = (recognition.getLeft() + recognition.getRight()) / 2;
//                    double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//                    telemetry.addData(""," ");
//                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
//                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//
//                    telemetry.addData("- Left/Right", "%.0f x %.0f", recognition.getLeft(), recognition.getRight());
//                    telemetry.update();
//                    sleep(1000);
//                    if (x > 30 && x < 300) {
//                        spikeMark = 2;
//                    } else if (x > 300) {
//                        spikeMark = 3;
//                    }
//                }
//            }   // end for() loop
            robot.turn(MAX_AUTO_TURN, -20, 3);

        } else {
                robot.turn(MAX_AUTO_TURN, -40, 3);
                currentRecognitions = getRecognitions(currentRecognitions);
                currentRecognitions = getRecognitions(currentRecognitions);
                currentRecognitions = getRecognitions(currentRecognitions);
                if (currentRecognitions.size() > 0) {
                    spikeMark = 3;
                }
                robot.turn(MAX_AUTO_TURN, 20, 3);
        }
        return spikeMark;

    }

    private List<Recognition> getRecognitions(List<Recognition> currentRecognitions) {
        if (currentRecognitions == null || currentRecognitions.size() == 0) {
            sleep(300);
            robot.tfod.getFreshRecognitions();
            currentRecognitions = robot.tfod.getRecognitions();
            if (currentRecognitions != null) {
                telemetry.addData("# Objects Detected " + iter++, currentRecognitions.size());
                telemetry.update();
            }
        }
        return currentRecognitions;
    }

    protected void autoDrive(boolean blueTeam, boolean isBackStage) throws InterruptedException {

//TODO       autonomous period that hopefully works and scores many points for us


        //close claws
        robot.setRightClawPositionAndDirection(1, Servo.Direction.FORWARD);
        robot.setLeftClawPositionAndDirection(1, Servo.Direction.REVERSE);

        // go forward 10 in
        robot.encoderDrive(MAX_AUTO_SPEED,17,17,8);

//        ///TESTING CODE TAKE OUT LATER
//        robot.turn(MAX_AUTO_TURN, 20, 3);
//        while (opModeIsActive() && !isStopRequested()) {
//            testSpikePos();
//        }
//        //TAKE OUT TESTING CODE ABOVE

        int spikePos = determineSpikePos();
        int currentRotation = spikePos == 3 ? 20 : 0;

        telemetry.addData("Spike Pos", spikePos);
        telemetry.addData("Current Rotation", currentRotation);

        telemetry.update();
//        sleep(2000);

        currentRotation += turnToSpikePos(spikePos,blueTeam,isBackStage);

        telemetry.addData("Current Rotation after turnToSpikePos", currentRotation);
        telemetry.update();
//        sleep(2000);

        robot.dropArmForPixelPickup();

        currentRotation += sweepToSpikePos(spikePos,blueTeam,isBackStage);
        telemetry.addData("Current Rotation after sweepToSpikePos", currentRotation);
        telemetry.update();

        // place pixel
        robot.setRightClawPositionAndDirection(RobotHardware.CLAW_OPEN_POSITION, Servo.Direction.FORWARD);
        telemetry.addData("Right Claw Pos", "%4.2f", RobotHardware.rightClaw.getPosition());
        telemetry.update();

        sleep(50);



// ----------------
        // base rotation up
        robot.rotationMotorSetPoint = -400;
        robot.runClosedLoops();
        while (RobotHardware.baseRotationMotor.getCurrentPosition() > -300 && opModeIsActive()) {
            robot.runClosedLoops();
            telemetry.addData("lifting arm", RobotHardware.baseRotationMotor.getCurrentPosition());
        }

        robot.detractArm();

        robot.setWristPositionAndDirection(RobotHardware.WRIST_PLACE_POSITION, Servo.Direction.FORWARD);

        robot.raiseOrLowerArm(-300,100);

        sleep (200);

        robot.raiseOrLowerArm(-200,100);

        // place pixel
        robot.setRightClawPositionAndDirection(RobotHardware.CLAW_CLOSED_POSITION, Servo.Direction.FORWARD);
        telemetry.addData("Right Claw Pos", "%4.2f", RobotHardware.rightClaw.getPosition());
        telemetry.update();

        if (blueTeam) {
            if (spikePos == 1) {
                // turn left
                robot.turn(MAX_AUTO_TURN, -120, 3);

//                robot.turn(MAX_AUTO_TURN, -90, 3);
//                robot.turn(MAX_AUTO_TURN, -30, 3);
            } else if (spikePos == 2) {
                // turn right
                robot.turn(MAX_AUTO_TURN, -90, 3);
            } else {
                // turn right
                robot.turn(MAX_AUTO_TURN, -50, 3);
            }

            //strafe right
            robot.encoderStrafe(1,-2,-2,1);

        } else {
            if (spikePos == 3) {
                // turn left
                robot.turn(MAX_AUTO_TURN, 140, 3);

//                robot.turn(MAX_AUTO_TURN, 90, 3);
//                robot.turn(MAX_AUTO_TURN, 30, 3);
            } else if (spikePos == 2) {
                // turn right
                robot.turn(MAX_AUTO_TURN, 90, 3);
            } else {
                // turn right
                robot.turn(MAX_AUTO_TURN, 50, 3);
            }
            //strafe left
            robot.encoderStrafe(1,2,2,1);
        }

        if (blueTeam) {
            //strafe left
            robot.encoderStrafe(0.5, 26, 26, 3);
            robot.encoderStrafe(0.5, -3, -3, 2);

            //if not backstage, drive forward
            if (isBackStage) {
                robot.encoderDrive(MAX_AUTO_SPEED, 40, 40, 5);
            } else {
                robot.encoderDrive(MAX_AUTO_SPEED, 95, 95, 10);
            }
        } else {

            //strafe left
            robot.encoderStrafe(0.5,-26,-26,3);
            robot.encoderStrafe(0.5,3,3,2);

            //if not backstage, drive forward
            if (isBackStage) {
                robot.encoderDrive(MAX_AUTO_SPEED, 40, 40, 5);
            } else {
                robot.encoderDrive(MAX_AUTO_SPEED, 95, 95, 10);
            }
        }
        while (opModeIsActive()) {}


//        boolean aprilTagMoveCompleted = false;
//        while (opModeIsActive() && !aprilTagMoveCompleted ) {
//            aprilTagMoveCompleted = driveToAprilTag(blueTeam, spikePos);
//            robot.runClosedLoops();
//            telemetry.addData("AprilTagCompleted", aprilTagMoveCompleted);
//            telemetry.update();
//        }
//
//        if (targetFound) {
//
//            robot.liftPixelForPlacement();
//
//            // drop pixel
//        robot.setLeftClawPositionAndDirection(RobotHardware.CLAW_OPEN_POSITION, Servo.Direction.REVERSE);// base rotation up
//            robot.raiseOrLowerArm(-400, 100);
//
////         rotate wrist
//            robot.setWristPositionAndDirection(RobotHardware.WRIST_UP_POSITION, Servo.Direction.FORWARD);
//
//            robot.detractArm();
//            // base rotation down
//            robot.raiseOrLowerArm(-300,100);
//
//            // base rotation down
//            robot.raiseOrLowerArm(-200,100);
//
//            // base rotation down
//            robot.raiseOrLowerArm(-150,100);

            if (blueTeam) {
                robot.encoderStrafe(MAX_AUTO_SPEED, -20, -20, 3);
            } else {
                robot.encoderStrafe(MAX_AUTO_SPEED, 20, 20, 3);
            }
            robot.encoderDrive(MAX_AUTO_SPEED,5,5,3);
//        }



        while (opModeIsActive()) {
            robot.runClosedLoops();

        }
    }

    private int turnToSpikePos(int spikePos,boolean blueTeam,boolean isBackStage) throws InterruptedException {
        int turnDirectionMultiplier = 1;
        if ((blueTeam && !isBackStage) || (!blueTeam && isBackStage)) {
            turnDirectionMultiplier = 1;
        }
        int degreesToTurn = -200 * turnDirectionMultiplier;
        if (spikePos == 1) {
            // turn right
            degreesToTurn = -180 * turnDirectionMultiplier;

//            robot.turn(MAX_AUTO_TURN, -90 * turnDirectionMultiplier, 3);
//            robot.turn(MAX_AUTO_TURN, -70 * turnDirectionMultiplier, 3);
        } else if (spikePos == 2) {
            // turn right
            robot.encoderDrive(MAX_AUTO_SPEED,3,3,3);

            degreesToTurn = -140 * turnDirectionMultiplier;

//            robot.turn(MAX_AUTO_TURN, -90 * turnDirectionMultiplier, 3);
//            robot.turn(MAX_AUTO_TURN, -90 * turnDirectionMultiplier, 3);


//            robot.turn(MAX_AUTO_TURN, 40 * turnDirectionMultiplier, 3);
        }
//        else {
            // turn left

//            robot.turn(MAX_AUTO_TURN, -90 * turnDirectionMultiplier, 3);
//            robot.turn(MAX_AUTO_TURN, -90 * turnDirectionMultiplier, 3);
//            robot.turn(MAX_AUTO_TURN, -10 * turnDirectionMultiplier, 3);
//        }
            robot.turn(MAX_AUTO_TURN, degreesToTurn , 3);

        return degreesToTurn;

    }

    private int sweepToSpikePos(int spikePos,boolean blueTeam,boolean isBackStage) throws InterruptedException {
        int turnDirectionMultiplier = 1;
        if ((blueTeam && !isBackStage) || (!blueTeam && isBackStage)) {
            turnDirectionMultiplier = 1;
        }
        int degreesToTurn = -35 * turnDirectionMultiplier;
        if (spikePos == 1) {
            // turn left
            degreesToTurn = 24 * turnDirectionMultiplier;j
        } else if (spikePos == 2) {
            // turn right
            degreesToTurn = -60 * turnDirectionMultiplier;
        }

        // turn left
        robot.turn(MAX_AUTO_TURN, degreesToTurn, 3);

        return degreesToTurn;
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        robot.setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
//    private void initAprilTag() {
//        // Create the AprilTag processor by using a builder.
//        aprilTag = new AprilTagProcessor.Builder().build();
//
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // eg: Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTag.setDecimation(2);
//
//        // Create the vision portal by using a builder.
//        if (USE_WEBCAM) {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .addProcessor(aprilTag)
//                    .build();
//        } else {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(BuiltinCameraDirection.BACK)
//                    .addProcessor(aprilTag)
//                    .build();
//        }
//    }



    public boolean driveToAprilTag (boolean blueTeam,int spikePos) {

        int desiredTagId = spikePos + (blueTeam ? 0 : 3);

        desiredTag  = null;
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if (detection.id == desiredTagId) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        if (targetFound && desiredTag != null) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.update();
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
        boolean positionCompleted = false;//drive + strafe + turn  0;
        return positionCompleted;
    }

}
