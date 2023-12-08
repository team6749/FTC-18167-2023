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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Basic: Omni Linear OpMode", group = "Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
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
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

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

            if (gamepad1.dpad_right || gamepad1.dpad_left) {
                shaftMotorPower = 1;

                shaftDirection = gamepad1.dpad_right ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
            }
            robot.setShaftPowerAndDirection(shaftMotorPower, shaftDirection);


            // Move Base Rotation Motor
//            double baseRotationMotorPower = 0.0;
//            DcMotorSimple.Direction baseRotationMotorDirection = DcMotorSimple.Direction.FORWARD;

//            if (gamepad1.dpad_up || gamepad1.dpad_down) {
//                baseRotationMotorPower = 1;
//
//                baseRotationMotorDirection = gamepad1.dpad_up ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
//            }
//            robot.setBaseRotationMotorPosAndDirection(baseRotationMotorPower, baseRotationMotorDirection);


            // Move Claws
            if (gamepad1.a || gamepad1.b) {
                double clawPosition = gamepad1.a ? RobotHardware.CLAW_OPEN_POSITION : RobotHardware.CLAW_CLOSED_POSITION;
                if (!gamepad1.right_bumper) {
                    robot.setRightClawPositionAndDirection(clawPosition, Servo.Direction.FORWARD);
                }
                if (!gamepad1.left_bumper) {
                    robot.setLeftClawPositionAndDirection(clawPosition, Servo.Direction.REVERSE);
                }
            }
            //wrist
            if (gamepad1.x || gamepad1.y) {
                double wristPosition = gamepad1.x ? RobotHardware.WRIST_DOWN_POSITION : RobotHardware.WRIST_UP_POSITION;
                Servo.Direction wristDirection = gamepad1.x ? Servo.Direction.FORWARD : Servo.Direction.FORWARD;
                robot.setWristPositionAndDirection(wristPosition, wristDirection);
                telemetry.addData("wrist", "wrist position: " + wristPosition);
            }

            if (gamepad1.dpad_up || gamepad1.dpad_down) {

//                DcMotorSimple.Direction dir = gamepad1.dpad_up ? DcMotorSimple.Direction.REVERSE: DcMotorSimple.Direction.FORWARD;
                if (gamepad1.dpad_up) {
                    robot.setBaseRotationMotorTarget(robot.BASE_ROTATION_PLACE);
                } else {
                    robot.setBaseRotationMotorTarget(robot.BASE_ROTATION_PICKUP);
                }
            }
//            } else {
//                robot.setBaseRotationPower(0);
//            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Right Get pos", "%4.2f", RobotHardware.rightClaw.getPosition());
            telemetry.addData("Left Get pos", "%4.2f", RobotHardware.leftClaw.getPosition());
            telemetry.addData("Base Pos", RobotHardware.baseRotationMotor.getCurrentPosition());
            telemetry.addData("rotation speed", RobotHardware.baseRotationMotor.getPower());
            telemetry.update();
        }
    }
}
