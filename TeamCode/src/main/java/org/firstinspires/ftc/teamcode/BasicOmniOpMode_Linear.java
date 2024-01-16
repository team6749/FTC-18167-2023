/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;

@TeleOp(name = "Basic: Omni Linear OpMode", group = "Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private boolean hasEndGameRun = false;
    private double startTime;

    @Override
    public void runOpMode() throws InterruptedException{
        robot.init();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral - yaw;
            double rightFrontPower = axial - lateral + yaw;
            double leftBackPower = axial - lateral - yaw;
            double rightBackPower = axial + lateral + yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            robot.setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            // Move Shaft
            double shaftMotorPower = 0.0;
            DcMotorSimple.Direction shaftDirection = DcMotorSimple.Direction.FORWARD;

            // shaft - move arm up/down
            if (gamepad1.dpad_right || gamepad1.dpad_left || gamepad2.dpad_right || gamepad2.dpad_left) {
                shaftMotorPower = 1;

                shaftDirection = gamepad1.dpad_right || gamepad2.dpad_right ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
            }
            robot.setShaftPowerAndDirection(shaftMotorPower, shaftDirection);

            // TODO: It makes more sense to switch base rotation and shaft movement controls (updown and leftright) Just a tought

            // Base rotation
            if ((gamepad1.dpad_up || gamepad1.dpad_down ||
                    gamepad2.dpad_up || gamepad2.dpad_down) && !gamepad1.back) {
                if (gamepad1.dpad_up) {
                    robot.liftPixelForPlacement();
                } else {
                    robot.dropArmForPixelPickup();
                }
//                sleep(10);
            }

            // Move Claws
            if (gamepad1.a || gamepad1.b || gamepad2.a || gamepad2.b) {
                double clawPosition = (gamepad1.b || gamepad2.b) ? RobotHardware.CLAW_OPEN_POSITION : RobotHardware.CLAW_CLOSED_POSITION;
                if (!gamepad1.right_bumper && !gamepad2.right_bumper) {
                    robot.setRightClawPositionAndDirection(clawPosition, Servo.Direction.FORWARD);
                }
                if (!gamepad1.left_bumper && !gamepad2.left_bumper) {
                    robot.setLeftClawPositionAndDirection(clawPosition, Servo.Direction.REVERSE);
                }
            }

            //wrist
            if (gamepad1.x || gamepad1.y || gamepad2.x || gamepad2.y) {
                double wristPosition = (gamepad1.x || gamepad2.x) ? RobotHardware.WRIST_PLACE_POSITION : RobotHardware.WRIST_PICKUP_POSITION;
                Servo.Direction wristDirection = (gamepad1.x || gamepad2.x) ? Servo.Direction.FORWARD : Servo.Direction.FORWARD;
                robot.setWristPositionAndDirection(wristPosition, wristDirection);

            }

            // Climbing function defined below :))))
//            if (gamepad1.back && !hasEndGameRun) {
//                // setpoint rotate
//                robot.raiseOrLowerArm(RobotHardware.BASE_ROTATION_CLIMB, 100);
//                robot.rotationMotorSetPoint = RobotHardware.BASE_ROTATION_CLIMB;
//                // wait til brm is within plus minus 50
//                boolean isUpperPressed = RobotHardware.upperLimitSwitch.isPressed();
//                int revs = 0;
//                if(Math.abs(RobotHardware.BASE_ROTATION_CLIMB - RobotHardware.baseRotationMotor.getCurrentPosition()) < 50) {                //make arm extend
//                   robot.extendArm();
//
//
//                    robot.darth();
//                    // worked!
//                    sleep(200);
//                    robot.encoderDrive(.7, -16.0, -16.0, 3);
//                    robot.darth();
//
//                    // maybe pause or something pls
//                    sleep(200);
//
//                    //pull robot up (arm considense)
//                    robot.detractArm();
//                    robot.darth();
//
//                    hasEndGameRun = true;
//                }
//            }
            if (gamepad1.back) {
                robot.raiseOrLowerArm(RobotHardware.BASE_ROTATION_CLIMB,100);

            }

            robot.runClosedLoops();


            if (gamepad1.left_stick_button == true) {
                robot.raiseOrLowerArm(-400,100);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Front left/right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Drive Back  left/right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            telemetry.addData("Right Claw Pos", "%4.2f", RobotHardware.rightClaw.getPosition());
            telemetry.addData("Left Claw Pos", "%4.2f", RobotHardware.leftClaw.getPosition());

            telemetry.addData("Wrist Position", RobotHardware.wrist.getPosition());
            telemetry.addData("Wrist Position set", "%4.2f", robot.wristStartPosition);

            telemetry.addData("Base Rotation Current Pos", RobotHardware.baseRotationMotor.getCurrentPosition());
            telemetry.addData("Base Rotation Target Pos", robot.rotationMotorSetPoint);
            telemetry.addData("Base Rotation Power", RobotHardware.baseRotationMotor.getPower());

            telemetry.addData("Shaft Power", RobotHardware.shaftMotor.getPower());
            telemetry.addData("Shaft Lower Limit Pressed", RobotHardware.lowerLimitSwitch.isPressed());
            telemetry.addData("ShaftUpperLimitPressed", RobotHardware.upperLimitSwitch.isPressed());

            telemetry.addData("Climbing Button Pressed", gamepad1.back);

            telemetry.addLine("             :)");

            telemetry.update();
        }
    }

}
