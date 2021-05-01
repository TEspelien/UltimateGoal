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
//
//package org.eastsideprep.ftc.teamcode.null8103;
//
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.util.Timing;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import java.util.concurrent.TimeUnit;
//
//
//@TeleOp(name = "shooter test")
//
////simple drive teleop
//public class shooter_test extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    RobotHardware robot = new RobotHardware();
//
//    @Override
//    public void runOpMode() {
//
//        robot.init(hardwareMap);
//        /* Initialize the hardware variables.
//         * The init() method of the hardware class does all the work here
//         */
//
//        telemetry.addData("Say", "Ready");
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//        GamepadEx gamepad = new GamepadEx(gamepad1);
//
//
//
//
//        while (opModeIsActive()) {
//
//            if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8) {
//                robot.shoot3Rings();
//            }
//            telemetry.addData("velo", robot.shooter.getCorrectedVelocity());
//
//            telemetry.update();
//        }
//    }
//}

