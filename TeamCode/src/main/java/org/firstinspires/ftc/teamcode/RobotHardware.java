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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotHardware {

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO = 0.5;
    public static final double CLAW_OPEN_POSITION = 1.0;
    public static final double CLAW_CLOSED_POSITION = 0;

    public static final double WRIST_DOWN_POSITION = 0;
    public static final double WRIST_UP_POSITION = 1.0;

    public static final int BASE_ROTATION_PICKUP = 0;

    public static final int BASE_ROTATION_PLACE = -3900;


    DcMotor shaftMotor = null;
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    //TODO: remember the SEMI-COLONS!
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public static DcMotor baseRotationMotor = null;

    // TODO: DEFINE ENCODER HERE
//    private DutyCycleEncoder

    private Servo wrist = null;
//    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static Servo leftClaw = null;
    public static Servo rightClaw = null;
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


        baseRotationMotor = myOpMode.hardwareMap.get(DcMotor.class, "baseRotationMotor");

        baseRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseRotationMotor.setTargetPosition(0);
        baseRotationMotor.setPower(1);

        baseRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        baseRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        baseRotationMotor.setTargetPosition(-50);

        shaftMotor = myOpMode.hardwareMap.get(DcMotor.class, "shaftMotor");
        // TODO: DEFINE ENCODER HERE
        //TODO: remember the SEMI-COLONS!

        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        leftClaw = myOpMode.hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = myOpMode.hardwareMap.get(Servo.class, "rightClaw");

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

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
        shaftMotor.setDirection(shaftDirection);
        shaftMotor.setPower(shaftMotorPower);
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

    public void setBaseRotationMotorPosAndDirection(int baseRotationMotorPos, DcMotorSimple.Direction baseRotationMotorDirection) {
        baseRotationMotor.setDirection(baseRotationMotorDirection);
        baseRotationMotor.setTargetPosition(baseRotationMotorPos);
        baseRotationMotor.setPower(1);

        baseRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (baseRotationMotor.isBusy()) {
            myOpMode.telemetry.addData("Current Position", baseRotationMotor.getCurrentPosition());
            myOpMode.telemetry.addData("Target Position", baseRotationMotorPos);
            myOpMode.telemetry.update();
        }
//        baseRotationMotor.setPower(1);
    }


    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
//    public void setHandPositions(double offset) {
//        offset = Range.clip(offset, -0.5, 0.5);
//        leftHand.setPosition(MID_SERVO + offset);
//        rightHand.setPosition(MID_SERVO - offset);
//    }
}
