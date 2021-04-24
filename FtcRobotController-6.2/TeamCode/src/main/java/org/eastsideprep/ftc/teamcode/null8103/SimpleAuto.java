package org.eastsideprep.ftc.teamcode.null8103;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(name = "Simple Auto")
public class SimpleAuto extends LinearOpMode {

    SimpleHardware robot = new SimpleHardware();

    public void driveForward(double power, double time) {
        robot.leftBackMotor.setPower(power);
        robot.leftFrontMotor.setPower(power);
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        sleep((long) time);
        robot.leftBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
    }

    public void turnright() {
        robot.leftBackMotor.setPower(-0.25); //have some wheels turn different directions so it goes left
        robot.leftFrontMotor.setPower(0.25);
        robot.rightBackMotor.setPower(0.25);
        robot.rightFrontMotor.setPower(-0.25);
    }

    public void turnrightslowly() {
        robot.leftBackMotor.setPower(-0.05); //have low power so it goes very slow
        robot.leftFrontMotor.setPower(0.05);
        robot.rightFrontMotor.setPower(0.05);
        robot.rightBackMotor.setPower(-0.05);
    }

    public void turnleft(double power, double time) {
        robot.leftFrontMotor.setPower(-power);
        robot.leftBackMotor.setPower(-power);
        robot.rightBackMotor.setPower(power);
        robot.rightFrontMotor.setPower(power);
        sleep((long) time);
        robot.leftBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
    }

    public void backwards() {
        robot.leftBackMotor.setPower(-0.25);
        robot.leftFrontMotor.setPower(-0.25);
        robot.rightFrontMotor.setPower(-0.25);
        robot.rightBackMotor.setPower(-0.25);
        //sleep(l * 58);
    }

    public void motorStop() {
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
    }

    public void stopmotors() {
        robot.rightBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
    }

    public void strafeleftslowly() {
        robot.rightFrontMotor.setPower(0.05);
        robot.rightBackMotor.setPower(-0.05);
        robot.leftFrontMotor.setPower(-0.05);
        robot.leftBackMotor.setPower(0.05);
    }

    public void straferightslowly() {
        robot.rightFrontMotor.setPower(-0.05);
        robot.rightBackMotor.setPower(0.05);
        robot.leftFrontMotor.setPower(0.05);
        robot.leftBackMotor.setPower(-0.05);
        //sleep(l*12*2000);
    }

    public void strafeleft() {
        robot.rightFrontMotor.setPower(0.5);
        robot.rightBackMotor.setPower(-0.5);
        robot.leftFrontMotor.setPower(-0.5);
        robot.leftBackMotor.setPower(0.5);
    }

    public void straferight() {
        robot.rightFrontMotor.setPower(-0.5);
        robot.rightBackMotor.setPower(0.5);
        robot.leftFrontMotor.setPower(0.5);
        robot.leftBackMotor.setPower(-0.5);
    }


    @Override
    public void runOpMode() {

        robot.init(hardwareMap); //load hardware from other program
        print("Ready to go");

        waitForStart();

        driveForward(1, 1500);

        turnleft(0.75, 2000);

        //driveForward(0.3, 1000);

        motorStop();
        print("finished");


        while (opModeIsActive()) {
        }


    }

    public void print(String text) {
        telemetry.addData("log", text);
        telemetry.update();
    }

}

