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

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.opencv.imgproc.Imgproc.cvtColor;

//simple skystone detection teleop, code from https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/SkystoneDeterminationExample.java
@Autonomous(name = "detecting skystones")

public class Auto_detecting_skystones extends LinearOpMode {

    @Override
    public void runOpMode() {

        //these scary lines open the camera streaming
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvInternalCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //set the pipeline written below to the camera
        SkystoneFinderPipeline skystoneFinder = new SkystoneFinderPipeline();
        phoneCam.setPipeline(skystoneFinder);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("The skystone is ", skystoneFinder.getResult());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(1000);
        }

    }

    private static class SkystoneFinderPipeline extends OpenCvPipeline {

        //to find the skystone:
        //look at three rectangles
        //convert from RGB to YCrCb
        //extract Cb channel (stones are yellow so lots of contrast in the blue channel against skystones)

        //colors (for drawing rectangles)

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(70, 98);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181, 98);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(290, 98);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;//small sample space ensures that only we only check the stone

        //the points needed to define the three rectangles
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;


        //this is separate from the main processing method because we want to set Cb once from the first frame
        //to ensure that the three regions don't change if/when Cb is re-written to
        @Override
        public void init(Mat firstFrame) {

            //openCV likes 2 params: input and output variable to set to
            Imgproc.cvtColor(firstFrame, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        String outputMessage;

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);

            //average over each of the three regions
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            //draw a rectangle over each region to see what is being processed
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // top left corner
                    region1_pointB, // bottom right corner
                    BLUE, // rectangle color
                    2); //rectangle line thickness
            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    BLUE,
                    2);
            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    BLUE,
                    2);

            int max = Math.max(avg1, Math.max(avg2, avg3));

            if (max == avg1) {
                outputMessage = "left";
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        new Point(REGION1_TOPLEFT_ANCHOR_POINT.x - 10, REGION1_TOPLEFT_ANCHOR_POINT.y - 10), //top left corner
                        new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + 30, REGION1_TOPLEFT_ANCHOR_POINT.y + 30), //bottom right corner
                        GREEN,  //rectangle color
                        2); //rectangle line thickness
            } else if (max == avg2) {
                outputMessage = "center";
                Imgproc.rectangle(
                        input,
                        new Point(REGION2_TOPLEFT_ANCHOR_POINT.x - 10, REGION2_TOPLEFT_ANCHOR_POINT.y - 10),
                        new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + 30, REGION2_TOPLEFT_ANCHOR_POINT.y + 30),
                        GREEN,
                        2);
            } else if (max == avg3) {
                outputMessage = "right";
                Imgproc.rectangle(
                        input,
                        new Point(REGION3_TOPLEFT_ANCHOR_POINT.x - 10, REGION3_TOPLEFT_ANCHOR_POINT.y - 10),
                        new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + 30, REGION3_TOPLEFT_ANCHOR_POINT.y + 30),
                        GREEN,
                        2);
            }

            return input;//camera input with the rectangles drawn on top
        }

        public String getResult() {
            return outputMessage;
        }
    }
}

