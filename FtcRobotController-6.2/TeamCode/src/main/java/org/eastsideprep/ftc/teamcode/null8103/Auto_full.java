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
import org.opencv.core.Point;
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

@Autonomous(name = "full auto")

public class Auto_full extends OpMode {

    RobotHardware robot;
    SampleMecanumDrive drive;

    //positions used in all paths:
    Pose2d startPose, goalShootingPose, powerShootingPose, secondWobblePose, endPose;

    //three trajectories, one of which will be built and followed
    TrajectoryBuilder trajZero, TrajOne, TrajFour;

    Trajectory chosenTraj;


    @Override
    public void init() {

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

        startPose = new Pose2d(-60, -24, Math.toRadians(150));

        goalShootingPose = new Pose2d(0, -36, Math.toRadians(210));
        powerShootingPose = new Pose2d(0, -28, Math.toRadians(220));

        secondWobblePose = new Pose2d(-36, -48, Math.toRadians(180));

        endPose = new Pose2d(12, -36, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectoryBuilder trajZero = drive.trajectoryBuilder(startPose)
                .splineTo(powerShootingPose.vec(), powerShootingPose.getHeading())
                .addDisplacementMarker(() -> {

                })
                .splineTo(new Vector2d(16, -64), 0)
                .addDisplacementMarker(() -> {
                    robot.lowerOpenWobble();
                })
                .splineTo(secondWobblePose.vec(), secondWobblePose.getHeading())
                .addDisplacementMarker(() -> {
                    robot.closeRaiseWobble();
                })
                .splineTo(new Vector2d(18, -56), 0)
                .addDisplacementMarker(() -> {
                    robot.lowerOpenWobble();
                })
                .splineTo(endPose.vec(), endPose.getHeading())
                .addDisplacementMarker(() -> {
                    robot.closeRaiseWobble();
                });

        //1 ring -> goal B
        //TODO: add spline to intake starter stack
        TrajectoryBuilder trajOne = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-20, -20), Math.toRadians(-80))//dont hit the starter stack
                .splineTo(powerShootingPose.vec(), powerShootingPose.getHeading())
                .addDisplacementMarker(() -> {
                    powerShotSequence();
                })
                .splineTo(new Vector2d(16, -64), 0)
                .addDisplacementMarker(() -> {
                    robot.lowerOpenWobble();
                })
                .splineTo(secondWobblePose.vec(), secondWobblePose.getHeading())
                .addDisplacementMarker(() -> {
                    robot.closeRaiseWobble();
                })
                .splineTo(new Vector2d(18, -56), 0)
                .addDisplacementMarker(() -> {
                    robot.lowerOpenWobble();
                })
                .splineTo(endPose.vec(), endPose.getHeading())
                .addDisplacementMarker(() -> {
                    robot.closeRaiseWobble();
                });

        TrajectoryBuilder trajFour = drive.trajectoryBuilder(startPose);

        int numRings = ringDetectorPipeline.getResult();

        if (numRings == 4) {
            chosenTraj = trajFour.build();
        } else if (numRings == 1) {
            chosenTraj = trajOne.build();
        } else {
            chosenTraj = trajZero.build();
        }

        drive.followTrajectoryAsync(chosenTraj);
    }

    @Override
    public void loop() {

        drive.update();
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
            turnLeft(750, 0.4);
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

    public void turnLeft(long time, double power) {
        Timing.Timer timer = new Timing.Timer(time, TimeUnit.MILLISECONDS);
        do {
            robot.leftFront.set(-power);
            robot.rightBack.set(power);
            robot.leftBack.set(-power);
            robot.rightFront.set(power);
        } while (!timer.done());
        robot.leftFront.set(0);
        robot.rightBack.set(0);
        robot.leftBack.set(0);
        robot.rightFront.set(0);

    }


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