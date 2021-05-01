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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

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

//simple skystone detection teleop, code from https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/SkystoneDeterminationExample.java
@Autonomous(name = "ring detection auto messed with")

public class Auto_ring_detection_with_movement extends LinearOpMode {

    RobotHardware robot;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {

        telemetry.addData("log", "starting...");
        telemetry.update();

        //these scary lines open the camera streaming
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvInternalCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //set the pipeline written below to the camera
        RingDetectorPipeline RingDetectorPipeline = new RingDetectorPipeline();
        phoneCam.setPipeline(RingDetectorPipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();


        robot = new RobotHardware();
        robot.init(hardwareMap);

        double shooterPower = 0;

        boolean shootingDone = false;

        double wobblePivotLow = 0.3;
        double wobblePivotHigh = 1;
        double wobbleGrabberClosed = 0.2;
        double wobbleGrabberOpen = 1;

        double ringPusherLow = 0.58; //not touching ring
        double ringPusherHigh = 0.39; //ring pushed into shooter

        robot.backwards(450, 0.7); //for example
        robot.turnLeft(50, 0.7);
        sleep(1000);

        int numRings = RingDetectorPipeline.getResult();
        telemetry.addData("num rings: ", RingDetectorPipeline.getResult());
        telemetry.update();

        // Don't burn CPU cycles busy-looping in this sample


        //TODO:
        //drive forwards to shooting position

        robot.turnLeft(500,0.7);
        robot.backwards(1250, 0.7);
        robot.turnRight(880, 0.7);
        robot.backwards(850,0.7);

        ElapsedTime shooterTimer = new ElapsedTime();

        while (!shootingDone) {
            shooterPower = 0.93;

            robot.top_intake1.set(0.3);
            robot.top_intake2.set(0.3);
            int startTime = 1800;

            if (shooterTimer.milliseconds() >= startTime && shooterTimer.milliseconds() < startTime+500) {
                robot.RingPushServo.setPosition(ringPusherHigh);
            } else if (shooterTimer.milliseconds() >= startTime+500 && shooterTimer.milliseconds() < startTime+1000) {
                robot.RingPushServo.setPosition(ringPusherLow);

            } else if (shooterTimer.milliseconds() >= startTime+1000 && shooterTimer.milliseconds() < startTime+1500) {
                robot.RingPushServo.setPosition(ringPusherHigh);
            } else if (shooterTimer.milliseconds() >= startTime+1500 && shooterTimer.milliseconds() < startTime+2500) {
                robot.RingPushServo.setPosition(ringPusherLow);

            } else if (shooterTimer.milliseconds() >= startTime+2000 && shooterTimer.milliseconds() < startTime+3500) {
                robot.RingPushServo.setPosition(ringPusherHigh);
            } else if (shooterTimer.milliseconds() >= startTime+4500) {
                robot.RingPushServo.setPosition(ringPusherLow);
                sleep(200);
                robot.RingPushServo.setPosition(ringPusherHigh);
                sleep(500);
                shootingDone = true;
            }
            robot.shooter.setVelocity(200);
        }

        robot.shooter.setVelocity(0);

        if(numRings==0) {
            robot.turnRight(475,0.7);
            robot.backwards(1500, 0.7);
            robot.wobblePivot.setPosition(wobblePivotHigh);
            sleep(1000);
            robot.wobbleGrabber.setPosition(wobbleGrabberClosed);
            sleep(1000);
        } else if (numRings == 1) {
            robot.turnRight(200,0.7);
            robot.backwards(1500, 0.7);
            robot.wobblePivot.setPosition(wobblePivotHigh);
            sleep(1000);
            robot.wobbleGrabber.setPosition(wobbleGrabberClosed);
            sleep(1000);
            robot.wobblePivot.setPosition(wobblePivotLow);
            sleep(1000);
            robot.forwards(1000, 0.7);
        } else {
            robot.turnRight(100, 0.7);
            robot.backwards(2500,0.7);
            robot.turnRight(450, 0.7);
            robot.wobblePivot.setPosition(wobblePivotHigh);
            sleep(1000);
            robot.wobbleGrabber.setPosition(wobbleGrabberClosed);
            sleep(1000);
            robot.wobblePivot.setPosition(wobblePivotLow);
            sleep(1000);
            robot.forwards(600, 0.7);
            robot.turnLeft(850, 0.7);
            robot.forwards(2800, 0.7);
            robot.backwards(1300, 0.7);
            robot.runIntake(1);
            robot.forwards(1500,0.7);
            robot.backwards(1500,0.7);
        }
    }

    static class RingDetectorPipeline extends OpenCvPipeline {

        int numRings = 0;
        int CAMERA_WIDTH = 320;

        //to tune:
        Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
        Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);

        double HORIZON = 0.65 * CAMERA_WIDTH;
        double MIN_WIDTH = 100;

        double BOUND_RATIO = 0.7;

        Mat output = new Mat();
        Mat mat = new Mat();

        @Override
        public Mat processFrame(Mat input) {

            output.release();
            output = new Mat();

            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

            // variable to store mask in
            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1);
            Core.inRange(mat, lowerOrange, upperOrange, mask);

            Core.bitwise_and(input, input, output, mask);

            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

            //draw the horizon line for tuning
            Imgproc.line(output, new Point(0, HORIZON), new Point(CAMERA_WIDTH, HORIZON), new Scalar(255, 0, 0), 3);

            //Imgproc.drawContours(output, contours, -1, new Scalar(0.0, 255.0, 0.0), 4);

            int maxWidth = 0;
            Rect maxRect = new Rect();
            for (MatOfPoint c : contours) {
                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                Rect rect = Imgproc.boundingRect(copy);

                int w = rect.width;
                // checking if the rectangle is below the horizon
                if (w > maxWidth && rect.y + rect.height < HORIZON) {
                    maxWidth = w;
                    maxRect = rect;
                }
                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            //draw the rectangle found above
            Imgproc.rectangle(output, maxRect, new Scalar(0, 0, 255));

            double aspectRatio = (double) maxRect.height / maxRect.width;

            //telemetry.addData("aspect", "" + aspectRatio);
            //telemetry.addData("maxWidth", "" + maxWidth);

            if (maxWidth >= MIN_WIDTH) {
                if (aspectRatio > BOUND_RATIO) {
                    numRings = 4;
                } else {
                    numRings = 1;
                }
            } else {
                numRings = 0;
            }
            return output;
        }

        int getResult() {
            return numRings;
        }
    }
}

