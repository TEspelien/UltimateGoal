///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.eastsideprep.ftc.teamcode.null8103;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.localization.Localizer;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.util.Timing;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
////import org.eastsideprep.ftc.teamcode.null8103.Auto_ring_detection.RingDetectorPipeline;
//import org.eastsideprep.ftc.teamcode.null8103.drive.DriveConstants;
//import org.eastsideprep.ftc.teamcode.null8103.drive.SampleMecanumDrive;
//import org.opencv.core.Core;
//import org.opencv.core.CvType;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name = "auto for comp")
//
//public class Auto_for_comp extends LinearOpMode {
//
//    RobotHardware robot;
//    SampleMecanumDrive drive;
//
//    int numRings = -1;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        waitForStart();
//
//        /* init steps:
//        - init hardware
//        - set up phone camera and vision pipeline
//        - see number of rings
//        - generate chosen path based on number of rings
//         */
//
//        robot = new RobotHardware();
//        robot.init(hardwareMap);
//
//
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvInternalCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//
//        RingDetectorPipeline ringDetectorPipeline = new RingDetectorPipeline();
//
//        phoneCam.setPipeline(ringDetectorPipeline);
//        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                //can remove later
//                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
//
//                telemetry.addData("num rings", "" + ringDetectorPipeline.getResult());
//                telemetry.update();
//            }
//        });
//
//        double shooterPower = 0;
//
//        boolean shootingDone = false;
//
//
//
//        ElapsedTime shooterTimer = new ElapsedTime();
//
//
//        double ringPusherLow = 0.52; //not touching ring
//        double ringPusherHigh = 0.39; //ring pushed into shooter
//
//        //TODO:
//        //drive forwards a tiny bit to get a better vision result
//        robot.backwards(400, 0.7); //for example
//        robot.turnLeft(50, 0.7);
//                //robot.turnLeft(100, 0.7);
//        //robot.turnRight(100, 0.7);
////
////        for (int i = 0; i < 30000; i++) {
////            telemetry.addData("LF", robot.leftFront.getCurrentPosition());
////            telemetry.addData("LB", robot.leftBack.getCurrentPosition());
////            telemetry.addData("RF", robot.rightFront.getCurrentPosition());
////            telemetry.addData("RB", robot.rightBack.getCurrentPosition());
////            telemetry.update();
////            sleep(1);
////        }
//
//        sleep(2000);
//
//        while (opModeIsActive()) {
//            numRings = ringDetectorPipeline.getResult();
//            telemetry.addData("num rings: ", numRings);
//            telemetry.update();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(1000);
//        }
//
//        //TODO:
//        //drive forwards to shooting position
//
//        robot.turnLeft(500,0.7);
//        robot.backwards(1400, 0.7);
//        robot.turnRight(950, 0.7);
//        robot.backwards(900,0.7);
//
//        while (!shootingDone) {
//
//            shooterPower = 0.97;
//
//            robot.top_intake1.set(0.25);
//            robot.top_intake2.set(0.25);
//
//            if (shooterTimer.milliseconds() >= 3500 && shooterTimer.milliseconds() < 4000) {
//                robot.RingPushServo.setPosition(ringPusherHigh);
//            } else if (shooterTimer.milliseconds() >= 4000 && shooterTimer.milliseconds() < 5000) {
//                robot.RingPushServo.setPosition(ringPusherLow);
//
//            } else if (shooterTimer.milliseconds() >= 5000 && shooterTimer.milliseconds() < 5500) {
//                robot.RingPushServo.setPosition(ringPusherHigh);
//            } else if (shooterTimer.milliseconds() >= 5500 && shooterTimer.milliseconds() < 6500) {
//                robot.RingPushServo.setPosition(ringPusherLow);
//
//            } else if (shooterTimer.milliseconds() >= 6500 && shooterTimer.milliseconds() < 7500) {
//                robot.RingPushServo.setPosition(ringPusherHigh);
//            } else if (shooterTimer.milliseconds() >= 7500 && shooterTimer.milliseconds() < 8000) {
//                robot.RingPushServo.setPosition(ringPusherLow);
//                shootingDone = true;
//            }
//            robot.shooter.setPower(shooterPower);
//        }
//
//        robot.turnRight(400,0.7);
//        robot.backwards(1500, 0.7);
//        robot.lowerOpenWobble();
//
//        sleep(25000);
//
//        while (!shootingDone) {
//
//            shooterPower = 0.97;
//
//            robot.top_intake1.set(0.25);
//            robot.top_intake2.set(0.25);
//
//            if (shooterTimer.milliseconds() >= 3500 && shooterTimer.milliseconds() < 4000) {
//                robot.RingPushServo.setPosition(ringPusherHigh);
//            } else if (shooterTimer.milliseconds() >= 4000 && shooterTimer.milliseconds() < 5000) {
//                robot.RingPushServo.setPosition(ringPusherLow);
//
//            } else if (shooterTimer.milliseconds() >= 5000 && shooterTimer.milliseconds() < 5500) {
//                robot.RingPushServo.setPosition(ringPusherHigh);
//            } else if (shooterTimer.milliseconds() >= 5500 && shooterTimer.milliseconds() < 6500) {
//                robot.RingPushServo.setPosition(ringPusherLow);
//
//            } else if (shooterTimer.milliseconds() >= 6500 && shooterTimer.milliseconds() < 7500) {
//                robot.RingPushServo.setPosition(ringPusherHigh);
//            } else if (shooterTimer.milliseconds() >= 7500 && shooterTimer.milliseconds() < 8000) {
//                robot.RingPushServo.setPosition(ringPusherLow);
//                shootingDone = true;
//            }
//            robot.shooter.setP(shooterPower);
//        }
//
//        switch (numRings) {
//            case 0:
//                //TODO
//                //drive to A
//
//                robot.lowerOpenWobble();
//
//                //TODO
//                //drive to park
//            case 1:
//                //TODO
//                //drive to B
//
//                robot.lowerOpenWobble();
//
//                //TODO
//                //drive to park
//            case 4:
//                //TODO
//                //drive to C
//                robot.lowerOpenWobble();
//
//                //TODO
//                //drive to park
//        }
//    }
//    static class RingDetectorPipeline extends OpenCvPipeline {
//
//        int numRings = 0;
//        int CAMERA_WIDTH = 320;
//
//        //to tune:
//        Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
//        Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);
//
//        double HORIZON = 0.65 * CAMERA_WIDTH;
//        double MIN_WIDTH = 100;
//
//        double BOUND_RATIO = 0.7;
//
//        Mat output = new Mat();
//        Mat mat = new Mat();
//
//        @Override
//        public Mat processFrame(Mat input) {
//
//            output.release();
//            output = new Mat();
//
//            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
//
//            // variable to store mask in
//            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1);
//            Core.inRange(mat, lowerOrange, upperOrange, mask);
//
//            Core.bitwise_and(input, input, output, mask);
//
//            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);
//
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
//
//            //draw the horizon line for tuning
//            Imgproc.line(output, new Point(0, HORIZON), new Point(CAMERA_WIDTH, HORIZON), new Scalar(255, 0, 0), 3);
//
//            //Imgproc.drawContours(output, contours, -1, new Scalar(0.0, 255.0, 0.0), 4);
//
//            int maxWidth = 0;
//            Rect maxRect = new Rect();
//            for (MatOfPoint c : contours) {
//                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
//                Rect rect = Imgproc.boundingRect(copy);
//
//                int w = rect.width;
//                // checking if the rectangle is below the horizon
//                if (w > maxWidth && rect.y + rect.height < HORIZON) {
//                    maxWidth = w;
//                    maxRect = rect;
//                }
//                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
//                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
//            }
//
//            //draw the rectangle found above
//            Imgproc.rectangle(output, maxRect, new Scalar(0, 0, 255));
//
//            double aspectRatio = (double) maxRect.height / maxRect.width;
//
//            //telemetry.addData("aspect", "" + aspectRatio);
//            //telemetry.addData("maxWidth", "" + maxWidth);
//
//            if (maxWidth >= MIN_WIDTH) {
//                if (aspectRatio > BOUND_RATIO) {
//                    numRings = 4;
//                } else {
//                    numRings = 1;
//                }
//            } else {
//                numRings = 0;
//            }
//            return output;
//        }
//
//        int getResult() {
//            return numRings;
//        }
//    }
//
//}