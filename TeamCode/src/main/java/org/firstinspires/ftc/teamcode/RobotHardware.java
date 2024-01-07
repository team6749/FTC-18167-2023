/* Copyright (c) 2022 FIRST. All rights reserved.
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

import static java.lang.Thread.sleep;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;


public class RobotHardware {

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double CLAW_OPEN_POSITION = 0;
    public static final double CLAW_CLOSED_POSITION = 1.0;
    private static AndroidSoundPool androidSoundPool;
    public static final double WRIST_UP_POSITION = 0;
    public static final double WRIST_PLACE_POSITION = 0;

    public static final double WRIST_PICKUP_POSITION = 1;

    public static final int BASE_ROTATION_PICKUP = 0;

    public static final int BASE_ROTATION_PLACE = -3300;

    public static final int BASE_ROTATION_CLIMB = -2048;


    //This is probably pretty close, but could be off
    public static final int LEVEL_OFFSET = -500;

    //This is just a guess, needs to be tested
    public static final double GRAVITY_FORCE = -0.195;

    public static DcMotor shaftMotor = null;

    public static TouchSensor lowerLimitSwitch = null;
    public static TouchSensor upperLimitSwitch = null;
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    //TODO: remember the SEMI-COLONS!
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public double wristStartPosition = 0;
    public static DcMotorEx baseRotationMotor = null;

//    private DutyCycleEncoder
    public double rotationMotorSetPoint = BASE_ROTATION_PICKUP;

    public static Servo wrist = null;
    public static Servo leftClaw = null;
    public static Servo rightClaw = null;
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: Core Hex Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.5625 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = 18.70129;//(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftfront_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftback_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightfront_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightback_drive");
        androidSoundPool = new AndroidSoundPool();
        androidSoundPool.initialize(SoundPlayer.getInstance());

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        baseRotationMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "brm");
        baseRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // use braking to slow the motor down faster
        baseRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        shaftMotor = myOpMode.hardwareMap.get(DcMotor.class, "shaftMotor");
        lowerLimitSwitch = myOpMode.hardwareMap.get(TouchSensor.class, "lowerLimitSwitch");
        upperLimitSwitch = myOpMode.hardwareMap.get(TouchSensor.class, "upperLimitSwitch");
        //TODO: remember the SEMI-COLONS!

        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        leftClaw = myOpMode.hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = myOpMode.hardwareMap.get(Servo.class, "rightClaw");

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();


//        while (!myOpMode.isStopRequested()) {
//            rotationMotorSetPoint = -500;
////            runBaseMotorClosedLoop();
//            runBaseMotorClosedLoopWithGravityStabilized();
//
//            myOpMode.telemetry.addData("Base Rotation Current Pos", RobotHardware.baseRotationMotor.getCurrentPosition());
//            myOpMode.telemetry.addData("Base Rotation Target Pos", RobotHardware.baseRotationMotor.getTargetPosition());
//            myOpMode.telemetry.addData("Base Rotation Power", RobotHardware.baseRotationMotor.getPower());
//
//            myOpMode.telemetry.update();
//
//        }
    }


    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontPower  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontPower Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackPower   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackPower  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }


    public void setShaftPowerAndDirection(double shaftMotorPower, DcMotorSimple.Direction shaftDirection) {
        if (lowerLimitSwitch.isPressed() && shaftDirection == DcMotorSimple.Direction.FORWARD) {
            shaftMotor.setPower(0.0);
        } else if(!upperLimitSwitch.isPressed() && shaftDirection == DcMotorSimple.Direction.REVERSE) {
            shaftMotor.setPower(0.0);
//        } else if(!upperLimitSwitch.isPressed() && !lowerLimitSwitch.isPressed()){
//            shaftMotor.setDirection(shaftDirection);
//            shaftMotor.setPower(shaftMotorPower);
        } else {
            shaftMotor.setDirection(shaftDirection);
            shaftMotor.setPower(shaftMotorPower);
        }
    }

    public void setWristPositionAndDirection(double wristPosition, Servo.Direction wristDirection) {
        wrist.setDirection(wristDirection);
        wrist.setPosition(wristPosition);
    }

    public void setLeftClawPositionAndDirection(double leftClawPosition, Servo.Direction leftClawDirection) {
        leftClaw.setDirection(leftClawDirection);
        leftClaw.setPosition(leftClawPosition);
    }

    public void setRightClawPositionAndDirection(double rightClawPosition, Servo.Direction rightClawDirection) {
        rightClaw.setDirection(rightClawDirection);
        rightClaw.setPosition(rightClawPosition);
     }
     public void runClosedLoops() {
         runBaseMotorClosedLoopWithGravityStabilized();
         runShaftMotorClosedLoop();
     }
    public void runBaseMotorClosedLoopWithGravityStabilized() {
        double p = 0.0015;
        double error = rotationMotorSetPoint - baseRotationMotor.getCurrentPosition();
        double cosOfCurrent = Math.cos(Math.toRadians(tprToDegrees(baseRotationMotor.getCurrentPosition() + LEVEL_OFFSET)));
        double calculated = ((cosOfCurrent * GRAVITY_FORCE) + (error * p));
        double maxPower = 0.5;
        baseRotationMotor.setPower(Math.min(Math.max(-maxPower, -calculated), maxPower));
        baseRotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runBaseMotorClosedLoop() {
        double p = 0.0015;
        double error = rotationMotorSetPoint - baseRotationMotor.getCurrentPosition();
        double calculated = (error * p);
        double maxPower = 0.45;
        baseRotationMotor.setPower(Math.min(Math.max(-maxPower, -calculated), maxPower));
        baseRotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runShaftMotorClosedLoop() {
        if (lowerLimitSwitch.isPressed() && shaftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
            shaftMotor.setPower(0.0);
        } else if(!upperLimitSwitch.isPressed() && shaftMotor.getDirection() == DcMotorSimple.Direction.REVERSE) {
            shaftMotor.setPower(0.0);
        }
    }


    public int tprToDegrees(int tpr){
        return (tpr / 8192) * 360;
    }

    public static int degreesToTpr(int deg){
        return (deg / 360) * 8192;
    }

    public static void darth(){
        androidSoundPool.play("RawRes:ss_power_up");
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);


//                    newLeftFrontTarget = leftFrontDrive.getCurrentPosition() - (int)(5*COUNTS_PER_MOTOR_REV);
//            newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int)(5*COUNTS_PER_MOTOR_REV);
//            newLeftBackTarget = leftBackDrive.getCurrentPosition() - (int)(5*COUNTS_PER_MOTOR_REV);
//            newRightBackTarget = rightBackDrive.getCurrentPosition() - (int)(5*COUNTS_PER_MOTOR_REV);

            encoderMove(speed, timeoutS, newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
        }
    }


    public void encoderStrafe(double speed,
                              double frontInches, double backInches,
                              double timeoutS) throws InterruptedException {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() - (int) (frontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int) (frontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() - (int) (backInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() - (int) (backInches * COUNTS_PER_INCH);

            encoderMove(speed, timeoutS, newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);

        }
    }

    private void encoderMove(double speed, double timeoutS, int newLeftFrontTarget, int newRightFrontTarget, int newLeftBackTarget, int newRightBackTarget) throws InterruptedException {
        leftFrontDrive.setTargetPosition(newLeftFrontTarget);
        rightFrontDrive.setTargetPosition(newRightFrontTarget);
        leftBackDrive.setTargetPosition(newLeftBackTarget);
        rightBackDrive.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftFrontDrive.setPower(Math.abs(speed));
        rightFrontDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (myOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftFrontDrive.isBusy() || rightFrontDrive.isBusy()
                || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Drive Front left/right", "%4.2f, %4.2f", leftFrontDrive.getPower(), rightFrontDrive.getPower());
            myOpMode.telemetry.addData("Drive Back  left/right", "%4.2f, %4.2f", leftBackDrive.getPower(), rightBackDrive.getPower());

            myOpMode.telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
            myOpMode.telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
                    leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        // Stop all motion;
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);   // optional pause after each move.
    }


    public void turn(double speed,
                     double degrees,
                             double timeoutS) throws InterruptedException {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // degrees to inches
        double inchesPerDegree = (20.8 * 3.14) / 360;

        double inchesToRotate = degrees * inchesPerDegree;



        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() - (int) (inchesToRotate * -1 * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int) (inchesToRotate * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() - (int) (inchesToRotate* -1 * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() - (int) (inchesToRotate * COUNTS_PER_INCH);

            encoderMove(speed, timeoutS, newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);

        }
    }
}
