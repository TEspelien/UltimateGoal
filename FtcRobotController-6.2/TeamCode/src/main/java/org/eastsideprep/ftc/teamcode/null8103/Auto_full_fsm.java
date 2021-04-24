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
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.eastsideprep.ftc.teamcode.null8103.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
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

        DRIVE_TO_PARK,

        IDLE //default and parked at the end
    }

    State currentState;

    RobotHardware robot;
    SampleMecanumDrive drive;

    //positions used in all paths:
    Pose2d startPose, visionPose, endPose;
    Pose2d intakeStartPose, intakeEndPose, highShootingPose, powerShootingPose;
    Pose2d regionA1Pose, regionA2Pose, regionB1Pose, regionB2Pose, regionC1Pose, regionC2Pose, secondWobblePose;

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

        startPose = new Pose2d(-60, -24, Math.toRadians(180));

        visionPose = new Pose2d(-36, -24, Math.toRadians(140));

        intakeStartPose = new Pose2d(-18, -32.5, Math.toRadians(-150));
        intakeEndPose = new Pose2d(-30, -39.5, Math.toRadians(-150));

        highShootingPose = new Pose2d(0, -36, Math.toRadians(160));
        powerShootingPose = new Pose2d(0, -28, Math.toRadians(170));

        secondWobblePose = new Pose2d(-38, -48, Math.toRadians(-90));

        regionA1Pose = new Pose2d(18, -66, Math.toRadians(30));
        regionA2Pose = new Pose2d(14, -62, Math.toRadians(30));

        regionB1Pose = new Pose2d(42, -42, Math.toRadians(-120));
        regionB2Pose = new Pose2d(38, -38, Math.toRadians(-120));

        regionC1Pose = new Pose2d(54, -66, Math.toRadians(-100));
        regionC2Pose = new Pose2d(50, -62, Math.toRadians(-100));

        endPose = new Pose2d(12, -24, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        driveToVision = drive.trajectoryBuilder(startPose)
                .splineTo(visionPose.vec(), visionPose.getHeading())
                .build();


        driveToA1 = drive.trajectoryBuilder(visionPose)
                .splineTo(regionA1Pose.vec(), regionA1Pose.getHeading())
                .build();
        driveTo2ndWobbleA = drive.trajectoryBuilder(driveToA1.end())
                .splineTo(secondWobblePose.vec(), secondWobblePose.getHeading())
                .build();
        driveToA2 = drive.trajectoryBuilder(driveTo2ndWobbleA.end())
                .splineTo(regionA2Pose.vec(), regionA2Pose.getHeading())
                .build();
        driveToPowerA = drive.trajectoryBuilder(driveToA2.end())
                .splineTo(powerShootingPose.vec(), powerShootingPose.getHeading())
                .build();


        driveToB1 = drive.trajectoryBuilder(visionPose)
                .splineTo(new Vector2d(-12, -12), Math.toRadians(-15))
                .splineTo(regionB1Pose.vec(), regionB1Pose.getHeading())
                .build();
        driveTo2ndWobbleB = drive.trajectoryBuilder(driveToB1.end())
                .splineTo(new Vector2d(0, -60), Math.toRadians(180))
                .splineTo(secondWobblePose.vec(), secondWobblePose.getHeading())
                .build();
        driveToB2 = drive.trajectoryBuilder(driveTo2ndWobbleB.end())
                .splineTo(new Vector2d(0, -60), Math.toRadians(180))
                .splineTo(regionB2Pose.vec(), regionB2Pose.getHeading())
                .build();
        driveToPowerB = drive.trajectoryBuilder(driveToB2.end())
                .splineTo(powerShootingPose.vec(), powerShootingPose.getHeading())
                .build();


        driveToC1 = drive.trajectoryBuilder(visionPose)
                .splineTo(new Vector2d(-12, -12), Math.toRadians(-30))
                .splineTo(regionC1Pose.vec(), regionC1Pose.getHeading())
                .build();
        driveTo2ndWobbleC = drive.trajectoryBuilder(driveToC1.end())
                .splineTo(secondWobblePose.vec(), secondWobblePose.getHeading())
                .build();
        driveToC2 = drive.trajectoryBuilder(driveTo2ndWobbleC.end())
                .splineTo(regionC2Pose.vec(), regionC2Pose.getHeading())
                .build();
        driveToPowerC = drive.trajectoryBuilder(driveToC2.end())
                .splineTo(powerShootingPose.vec(), powerShootingPose.getHeading())
                .build();

        driveAndIntake = drive.trajectoryBuilder(powerShootingPose)
                .splineTo(intakeStartPose.vec(), intakeStartPose.getHeading())
                .addDisplacementMarker(() -> {
                    robot.runIntake(1);
                })
                .splineTo(intakeEndPose.vec(), intakeEndPose.getHeading())
                //drive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),//why does this not workkk
                //drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.stopIntake();
                })
                .build();

        driveToHigh = drive.trajectoryBuilder(intakeEndPose)//this starting pose could be sus
                .splineTo(highShootingPose.vec(), highShootingPose.getHeading())
                .build();

        driveToEnd = drive.trajectoryBuilder(driveToHigh.end())
                .splineTo(endPose.vec(), endPose.getHeading())
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
                    }
                    break;
            }

            drive.update();
        }
    }


    Timing.Timer powerShotTimer = new Timing.Timer(3500, TimeUnit.MILLISECONDS);
    double pusherEnd = 0.75;

    public void powerShotSequence() {
        telemetry.addData("log", "shoot power shots!");
        robot.shooter.set(0.9);
        robot.RingPushServo.setPosition(0);
        powerShotTimer.start();
        if (powerShotTimer.currentTime() > 2000) {
            robot.RingPushServo.setPosition(pusherEnd);//shoot first ring
        } else if (powerShotTimer.currentTime() > 2500) {
            robot.RingPushServo.setPosition(0);
            //turnLeft(750, 0.4);
        } else if (powerShotTimer.currentTime() > 2750) {
            robot.RingPushServo.setPosition(pusherEnd);//shoot second ring

        } else if (powerShotTimer.currentTime() > 3000) {
            robot.RingPushServo.setPosition(0);
        } else if (powerShotTimer.currentTime() > 3250) {
            robot.RingPushServo.setPosition(0.8);

        } else if (powerShotTimer.done()) {
            robot.RingPushServo.setPosition(0);
            robot.shooter.set(0);
        }
    }
    //TODO: update this class
    class RingDetectorPipeline extends OpenCvPipeline {

        int numRings = 0;
        int CAMERA_WIDTH = 320;

        //to tune:
        Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
        Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);

        double HORIZON = 0.3 * CAMERA_WIDTH;
        double MIN_WIDTH = 0.2 * CAMERA_WIDTH;

        double BOUND_RATIO = 0.65;

        Mat matYCrCb = new Mat();
        Mat workingMat = new Mat();

        @Override
        public Mat processFrame(Mat input) {

            workingMat.release();
            workingMat = new Mat();
            //matYCrCb.release();

            Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

            // variable to store mask in
            Mat mask = new Mat(matYCrCb.rows(), matYCrCb.cols(), CvType.CV_8UC1);
            Core.inRange(matYCrCb, lowerOrange, upperOrange, mask);

            Core.bitwise_and(input, input, workingMat, mask);

            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

            //Imgproc.drawContours(workingMat, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);

            int maxWidth = 0;
            Rect maxRect = new Rect();
            for (MatOfPoint c : contours) {
                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                Rect rect = Imgproc.boundingRect(copy);

                int w = rect.width;
                // checking if the rectangle is below the horizon
                if (w > maxWidth && rect.y + rect.height > HORIZON) {
                    maxWidth = w;
                    maxRect = rect;
                }
                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            double aspectRatio = (double) maxRect.width / maxRect.height;

            //telemetry.addData("log", "" + aspectRatio);

            if (maxWidth >= MIN_WIDTH) {
                if (aspectRatio > BOUND_RATIO) {
                    numRings = 4;
                } else {
                    numRings = 1;
                }
            } else {
                numRings = 0;
            }
            return workingMat;
        }

        int getResult() {
            return numRings;
        }
    }
}