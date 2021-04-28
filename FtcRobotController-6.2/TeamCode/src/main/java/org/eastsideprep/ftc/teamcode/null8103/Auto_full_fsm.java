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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.eastsideprep.ftc.teamcode.null8103.Auto_ring_detection.RingDetectorPipeline;
import org.eastsideprep.ftc.teamcode.null8103.TunedConstants;
import org.eastsideprep.ftc.teamcode.null8103.drive.DriveConstants;
import org.eastsideprep.ftc.teamcode.null8103.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "full FSM auto")

public class Auto_full_fsm extends LinearOpMode {

    enum State {
        DRIVE_TO_VISION,
        VISION, //detecting number of rings, not moving

        DRIVE_TO_A1, //each possible driving segment
        DRIVE_TO_2nd_WOBBLE_A,
        DRIVE_TO_A2,
        DRIVE_TO_POWER_A,

        DRIVE_TO_B1,
        DRIVE_TO_2nd_WOBBLE_B,
        DRIVE_TO_B2,
        DRIVE_TO_POWER_B,

        DRIVE_TO_C1,
        DRIVE_TO_2nd_WOBBLE_C,
        DRIVE_TO_C2,
        DRIVE_TO_POWER_C,

        RELEASE_WOBBLE, //servo movements
        GRAB_WOBBLE,

        INTAKING, //moving slowly, intake powered

        DRIVE_TO_HIGH,
        SHOOT_POWER, //shooting at power shots
        SHOOT_HIGH, //shooting at high goal

        DRIVE_TO_END,

        IDLE //default and parked at the end
    }

    State currentState;

    RobotHardware robot;
    SampleMecanumDrive drive;

    Trajectory driveToVision;
    Trajectory driveToA1, driveTo2ndWobbleA, driveToA2, driveToPowerA;
    Trajectory driveToB1, driveTo2ndWobbleB, driveToB2, driveToPowerB;
    Trajectory driveToC1, driveTo2ndWobbleC, driveToC2, driveToPowerC;
    Trajectory driveAndIntake, driveToHigh, driveToEnd;

    int numRings = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        /* init steps:
        - init hardware
        - set up phone camera and vision pipeline
        - see number of rings
        - generate chosen path based on number of rings
         */

        robot = new RobotHardware();
        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvInternalCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        RingDetectorPipeline ringDetectorPipeline = new RingDetectorPipeline();

        phoneCam.setPipeline(ringDetectorPipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //can remove later
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

                telemetry.addData("num rings", "" + ringDetectorPipeline.getResult());
                telemetry.update();
            }
        });

        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d currentPose = TunedConstants.startPose;
        Localizer localizer = drive.getLocalizer();

        //things to tune:

        Timing.Timer powerShotTimer = new Timing.Timer(3000, TimeUnit.MILLISECONDS); //tune this length

        double shooterPower = 0;


        //other stuff:


        drive.setPoseEstimate(TunedConstants.startPose);

        driveToVision = drive.trajectoryBuilder(TunedConstants.startPose)
                .splineTo(TunedConstants.visionPose.vec(), TunedConstants.visionPose.getHeading())
                .build();


        driveToA1 = drive.trajectoryBuilder(TunedConstants.visionPose)
                .splineTo(TunedConstants.regionA1Pose.vec(), TunedConstants.regionA1Pose.getHeading())
                .build();
        driveTo2ndWobbleA = drive.trajectoryBuilder(driveToA1.end())
                .splineTo(TunedConstants.secondWobblePose.vec(), TunedConstants.secondWobblePose.getHeading())
                .build();
        driveToA2 = drive.trajectoryBuilder(driveTo2ndWobbleA.end())
                .splineTo(TunedConstants.regionA2Pose.vec(), TunedConstants.regionA2Pose.getHeading())
                .build();
        driveToPowerA = drive.trajectoryBuilder(driveToA2.end())
                .splineTo(TunedConstants.powerShootingStartPose.vec(), TunedConstants.powerShootingStartPose.getHeading())
                .build();


        driveToB1 = drive.trajectoryBuilder(TunedConstants.visionPose)
                .splineTo(new Vector2d(-12, -12), Math.toRadians(-15))
                .splineTo(TunedConstants.regionB1Pose.vec(), TunedConstants.regionB1Pose.getHeading())
                .build();
        driveTo2ndWobbleB = drive.trajectoryBuilder(driveToB1.end())
                .splineTo(new Vector2d(0, -60), Math.toRadians(180))
                .splineTo(TunedConstants.secondWobblePose.vec(), TunedConstants.secondWobblePose.getHeading())
                .build();
        driveToB2 = drive.trajectoryBuilder(driveTo2ndWobbleB.end())
                .splineTo(new Vector2d(0, -60), Math.toRadians(180))
                .splineTo(TunedConstants.regionB2Pose.vec(), TunedConstants.regionB2Pose.getHeading())
                .build();
        driveToPowerB = drive.trajectoryBuilder(driveToB2.end())
                .splineTo(TunedConstants.powerShootingStartPose.vec(), TunedConstants.powerShootingStartPose.getHeading())
                .build();


        driveToC1 = drive.trajectoryBuilder(TunedConstants.visionPose)
                .splineTo(new Vector2d(-12, -12), Math.toRadians(-30))
                .splineTo(TunedConstants.regionC1Pose.vec(), TunedConstants.regionC1Pose.getHeading())
                .build();
        driveTo2ndWobbleC = drive.trajectoryBuilder(driveToC1.end())
                .splineTo(TunedConstants.secondWobblePose.vec(), TunedConstants.secondWobblePose.getHeading())
                .build();
        driveToC2 = drive.trajectoryBuilder(driveTo2ndWobbleC.end())
                .splineTo(TunedConstants.regionC2Pose.vec(), TunedConstants.regionC2Pose.getHeading())
                .build();
        driveToPowerC = drive.trajectoryBuilder(driveToC2.end())
                .splineTo(TunedConstants.powerShootingStartPose.vec(), TunedConstants.powerShootingStartPose.getHeading())
                .build();

        driveAndIntake = drive.trajectoryBuilder(TunedConstants.powerShootingEndPose)
                .splineTo(TunedConstants.intakeStartPose.vec(), TunedConstants.intakeStartPose.getHeading())
                .addDisplacementMarker(() -> {
                    robot.runIntake(1);
                })
                .splineTo(TunedConstants.intakeEndPose.vec(), TunedConstants.intakeEndPose.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.stopIntake();
                })
                .build();

        driveToHigh = drive.trajectoryBuilder(TunedConstants.intakeEndPose)//this starting pose could be sus
                .splineTo(TunedConstants.highShootingPose.vec(), TunedConstants.highShootingPose.getHeading())
                .build();

        driveToEnd = drive.trajectoryBuilder(driveToHigh.end())
                .splineTo(TunedConstants.endPose.vec(), TunedConstants.endPose.getHeading())
                .build();

        currentState = State.IDLE;


        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {
                case DRIVE_TO_VISION:
                    if (!drive.isBusy()) {
                        currentState = State.VISION;
                        drive.followTrajectoryAsync(driveToVision);
                    }
                    break;
                case VISION:
                    if (!drive.isBusy()) {
                        numRings = ringDetectorPipeline.getResult();

                        if (numRings == 0) {
                            currentState = State.DRIVE_TO_A1;
                        } else if (numRings == 1) {
                            currentState = State.DRIVE_TO_B1;
                        } else {
                            currentState = State.DRIVE_TO_C1;
                        }
                    }
                    break;

                // 0 ring case
                case DRIVE_TO_A1:
                    if (!drive.isBusy()) {
                        robot.lowerOpenWobble();
                        drive.followTrajectoryAsync(driveTo2ndWobbleA);
                        currentState = State.DRIVE_TO_2nd_WOBBLE_A;
                    }
                    break;
                case DRIVE_TO_2nd_WOBBLE_A:
                    if (!drive.isBusy()) {
                        robot.closeRaiseWobble();
                        drive.followTrajectoryAsync(driveToA2);
                        currentState = State.DRIVE_TO_A2;
                    }
                    break;
                case DRIVE_TO_A2:
                    if (!drive.isBusy()) {
                        robot.lowerOpenWobble();
                        drive.followTrajectoryAsync(driveToPowerA);
                        currentState = State.SHOOT_POWER;
                    }
                    break;

                // 1 ring case
                case DRIVE_TO_B1:
                    if (!drive.isBusy()) {
                        robot.lowerOpenWobble();
                        drive.followTrajectoryAsync(driveTo2ndWobbleB);
                        currentState = State.DRIVE_TO_2nd_WOBBLE_B;
                    }
                    break;
                case DRIVE_TO_2nd_WOBBLE_B:
                    if (!drive.isBusy()) {
                        robot.closeRaiseWobble();
                        drive.followTrajectoryAsync(driveToB2);
                        currentState = State.DRIVE_TO_B2;
                    }
                    break;
                case DRIVE_TO_B2:
                    if (!drive.isBusy()) {
                        robot.lowerOpenWobble();
                        drive.followTrajectoryAsync(driveToPowerB);
                        currentState = State.SHOOT_POWER;
                    }
                    break;

                // 4 ring case
                case DRIVE_TO_C1:
                    if (!drive.isBusy()) {
                        robot.lowerOpenWobble();
                        drive.followTrajectoryAsync(driveTo2ndWobbleC);
                        currentState = State.DRIVE_TO_2nd_WOBBLE_C;
                    }
                    break;
                case DRIVE_TO_2nd_WOBBLE_C:
                    if (!drive.isBusy()) {
                        robot.closeRaiseWobble();
                        drive.followTrajectoryAsync(driveToC2);
                        currentState = State.DRIVE_TO_C2;
                    }
                    break;
                case DRIVE_TO_C2:
                    if (!drive.isBusy()) {
                        robot.lowerOpenWobble();
                        drive.followTrajectoryAsync(driveToPowerC);
                        currentState = State.SHOOT_POWER;
                    }
                    break;

                //TODO: tune these times
                case SHOOT_POWER:
                    if (!drive.isBusy()) {
                        shooterPower = 0.9;
                        if (powerShotTimer.currentTime() > 2000) {//wait a bit to spin up before shooting first ring
                            robot.pushRing();
                        } else if (powerShotTimer.currentTime() > 3000) { //after first shot, tune this time
                            drive.turnAsync(TunedConstants.POWER_SHOT_ANGLE);
                        } else if (powerShotTimer.currentTime() > 4000) {//shoot second ring
                            robot.pushRing();
                        } else if (powerShotTimer.currentTime() > 5000) { //after second shot, tune this time too
                            drive.turnAsync(TunedConstants.POWER_SHOT_ANGLE);
                        } else if (powerShotTimer.currentTime() > 6000) {//shoot third ring
                            robot.pushRing();
                        } else if (powerShotTimer.done()) {
                            currentState = State.INTAKING;
                            drive.followTrajectoryAsync(driveAndIntake);
                        }
                    }
                    break;

                case INTAKING:
                    if (!drive.isBusy()) {
                        currentState = State.DRIVE_TO_HIGH;
                        drive.followTrajectoryAsync(driveToHigh);
                    }
                    break;

                case DRIVE_TO_HIGH:
                    if (!drive.isBusy()) {
                        currentState = State.SHOOT_HIGH;
                        shooterPower = 0.95;
                        if (powerShotTimer.currentTime() > 2000) {//wait a bit to spin up before shooting first ring
                            robot.pushRing();
                        } else if (powerShotTimer.currentTime() > 3000) {//shoot second ring
                            robot.pushRing();
                        } else if (powerShotTimer.currentTime() > 4000) {//shoot third ring
                            robot.pushRing();
                        } else if (powerShotTimer.done()) {
                            currentState = State.DRIVE_TO_END;
                            drive.followTrajectoryAsync(driveToEnd);
                        }
                    }
                case DRIVE_TO_END:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }

            }

            //do this every loop no matter the state

            drive.update();

            if (shooterPower > 0) {
                //this should calculate the correct shooter power to maintain a constant velocity
                robot.shooter.set(shooterPower);
            }


            telemetry.addData("STATE", currentState);

            currentPose = localizer.getPoseEstimate();
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("heading", currentPose.getHeading());

            telemetry.update();
        }
    }
}