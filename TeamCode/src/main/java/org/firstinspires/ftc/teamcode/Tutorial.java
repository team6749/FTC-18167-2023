package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="autoTest")
public class Tutorial extends OpMode {

    private ElapsedTime     runtime = new ElapsedTime();
    DcMotor motor1;

    Servo servo1;

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);

        servo1 = hardwareMap.servo.get("jimmy");

    }

    @Override
    public void loop() {


        runtime.reset();
        while (runtime.seconds() < 3.0) {
            motor1.setPower(0.2);

        }
        servo1.setDirection(Servo.Direction.FORWARD);
        servo1.setPosition(0.2);

        runtime.reset();
        while (runtime.seconds() < 2.0) {
            motor1.setPower(0.1);
        }
        servo1.setDirection(Servo.Direction.FORWARD);
        servo1.setPosition(0.8);
        runtime.reset();
        while (runtime.seconds() < 1.0) {
            motor1.setPower(0.5);
        }

        servo1.setDirection(Servo.Direction.REVERSE);
        servo1.setPosition(0.1);
    }
}