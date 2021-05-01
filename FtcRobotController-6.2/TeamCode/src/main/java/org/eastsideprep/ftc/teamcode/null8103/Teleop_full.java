/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.eastsideprep.ftc.teamcode.null8103;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.eastsideprep.ftc.teamcode.null8103.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "Full teleop")

//simple drive teleop
public class Teleop_full extends LinearOpMode {
    int shooterVel = 0;

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        telemetry.addData(">", "Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        MecanumDrive mecanumDrive = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack);
        GamepadEx gamepad = new GamepadEx(gamepad1);
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        //double speedControl;

        //both of these are tuned manually to driver preference
        //used in the ether function to map gamepad -> drivetrain controls
        double driveGain = 0.6;
        double turnGain = 0.3;

        double ySpeed, xSpeed, turnSpeed, gyroAngle;
        // run until the end of the match (driver presses STOP)

        boolean isIntakeOn = false;
        boolean isShooterOn = false;
        boolean areBlockersOut = false;
        boolean wobbleDown = false;
        boolean grabberOpen = false;

        double shooterPower = 0;

        ElapsedTime shooterTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        robot.RingPushServo.setPosition(ringPusherLow);

        while (opModeIsActive()) {

            xSpeed = ether(gamepad.getLeftX(), driveGain);
            ySpeed = ether(gamepad.getLeftY(), driveGain);
            turnSpeed = ether(gamepad.getRightX(), turnGain);
            //gyroAngle = Math.toDegrees(robot.revIMU.getAbsoluteHeading() * Math.PI + Math.PI);//imu data is a double from -1 to 1, convert to 0 to 2pi
            mecanumDrive.driveRobotCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), true);
            //mecanumDrive.driveFieldCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), drive.getExternalHeading(), true);//squaring inputs is more precise
            //telemetry.addData("imu data", gyroAngle);

            if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.8) {
                robot.runIntake(1);
            } else if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.8) {
                robot.stopIntake();
            }
            if (gamepad.getButton(GamepadKeys.Button.B)) {
                robot.runIntake(-1); //run in reverse to unjam
            }

            if (gamepad.getButton(GamepadKeys.Button.X)) {
                robot.RingPushServo.setPosition(ringPusherHigh);
                sleep(200);
            } else if (gamepad.getButton(GamepadKeys.Button.Y)) {
                robot.RingPushServo.setPosition(ringPusherLow);
                sleep(200);
            }

            //left and right are flipped
            if (gamepad.getButton((GamepadKeys.Button.A)) && areBlockersOut) {
                robot.leftBlocker.setPosition(0.5);
                robot.rightBlocker.setPosition(0.8);
                sleep(200);
                areBlockersOut = false;
            } else if (gamepad.getButton((GamepadKeys.Button.A)) && !areBlockersOut) {
                robot.leftBlocker.setPosition(0.9);
                robot.rightBlocker.setPosition(0.4);
                sleep(200);
                areBlockersOut = true;
            }

            if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) && wobbleDown) {
                robot.wobblePivot.setPosition(robot.wobblePivotHigh);
                sleep(200);
                wobbleDown = false;
            } else if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) && !wobbleDown) {
                robot.wobblePivot.setPosition(robot.wobblePivotLow);
                sleep(200);
                wobbleDown = true;
            }

            if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) && grabberOpen) {
                robot.wobbleGrabber.setPosition(robot.wobbleGrabberClosed);
                sleep(200);
                grabberOpen = false;
            } else if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) && !grabberOpen) {
                robot.wobbleGrabber.setPosition(robot.wobbleGrabberOpen);
                sleep(200);
                grabberOpen = true;
            }

            if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8) {

                shooterVel = 3000;
                if (!isShooterOn) {
                    shooterTimer.reset();
                    isShooterOn = true;
                }

                robot.top_intake1.set(0.25);
                robot.top_intake2.set(0.25);


                if (shooterTimer.milliseconds() >= 3500 && shooterTimer.milliseconds() < 4000) {
                    robot.RingPushServo.setPosition(ringPusherHigh);
                } else if (shooterTimer.milliseconds() >= 4000 && shooterTimer.milliseconds() < 5000) {
                    robot.RingPushServo.setPosition(ringPusherLow);

                } else if (shooterTimer.milliseconds() >= 5000 && shooterTimer.milliseconds() < 5500) {
                    robot.RingPushServo.setPosition(ringPusherHigh);
                } else if (shooterTimer.milliseconds() >= 5500 && shooterTimer.milliseconds() < 6500) {
                    robot.RingPushServo.setPosition(ringPusherLow);

                } else if (shooterTimer.milliseconds() >= 6500 && shooterTimer.milliseconds() < 7500) {
                    robot.RingPushServo.setPosition(ringPusherHigh);
                } else if (shooterTimer.milliseconds() >= 7500 && shooterTimer.milliseconds() < 8000) {
                    robot.RingPushServo.setPosition(ringPusherLow);
                }
            } else if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.8) {
                shooterVel = 0;
                isShooterOn = false;
                robot.RingPushServo.setPosition(ringPusherLow);
            }

            if (shooterVel != 0) {
                robot.shooter.setVelocity(shooterVel);
            } else {
                robot.shooter.setPower(0);
            }

            telemetry.addData("shooter speed", robot.shooter.getVelocity());
            telemetry.addData("shooter timer", shooterTimer.milliseconds());

            telemetry.update();

            //sleep(25);
        }
    }


    //see graph at https://www.desmos.com/calculator/dx7yql2ekh
    public double ether(double x, double p) {
        double min = 0.2; //this means that very small joystick movements give enough power to overcome friction
        return Math.min(min + (1 - min) * (p * Math.pow(x, 3) + (1 - p) * x), 1);//max power is 1
    }


    double ringPusherLow = 0.52; //not touching ring
    double ringPusherHigh = 0.39; //ring pushed into shooter
}

