/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.compvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.OpenCvCamera;

import java.nio.charset.StandardCharsets;

@TeleOp
public class ThreeWebcams extends LinearOpMode
{
    WebcamName webcam1;
    WebcamName webcam2;
    WebcamName webcam3;
    OpenCvSwitchableWebcam switchableWebcam;

    @Override
    public void runOpMode() throws InterruptedException
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        webcam3 = hardwareMap.get(WebcamName.class, "Webcam 3");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /**
         * Here we use a special factory method that accepts multiple WebcamName arguments. It returns an
         * {@link OpenCvSwitchableWebcam} which contains a couple extra methods over simply an {@link OpenCvCamera}.
         */
        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam2, webcam3);

        //Define/switchup the pipelines - aka what each of the cams do
        SamplePipeline[] camPipeline = { new SamplePipeline(), new SamplePipeline(), new SamplePipeline() };


        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                String name = switchableWebcam.getActiveCamera().getDeviceName();

                int id = Integer.valueOf(name.charAt(7));

                switchableWebcam.setPipeline(camPipeline[id]);
                switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        int currentWebcam = 1;

        while (opModeIsActive())
        {
            telemetry.addLine("a = Webcam 1, b = Webcam 2, y = Webcam 3\n");
            telemetry.addData("Frame Count", switchableWebcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", switchableWebcam.getFps()));
            telemetry.addData("Total frame time ms", switchableWebcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", switchableWebcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", switchableWebcam.getOverheadTimeMs());
            telemetry.addData("Current webcam: ", currentWebcam);
            telemetry.update();

            /**
             * To switch the active camera, simply call
             * {@link OpenCvSwitchableWebcam#setActiveCamera(WebcamName)}
             */
            if(gamepad1.a)
            {
                currentWebcam = 1;
                switchableWebcam.setActiveCamera(webcam1);
            }
            else if(gamepad1.b)
            {
                currentWebcam = 2;
                switchableWebcam.setActiveCamera(webcam2);
            } else if(gamepad1.y)
            {
                currentWebcam = 3;
                switchableWebcam.setActiveCamera(webcam3);
            }

            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused = false;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */

            int green = 0;
            int tot = 0;
            for (int x = input.cols()/4; x < input.cols()*3/4; x++) {
                for (int y = input.rows()/4; y < input.rows()*3/4; y++ ) {
                    double[] d = input.get(x, y);
                    green += d[1];
                    tot++;
                }
            }


            if (green/tot > 200) {
                Imgproc.rectangle(
                        input,
                        new Point(
                                input.cols()/4,
                                input.rows()/4),
                        new Point(
                                input.cols()*(3f/4f),
                                input.rows()*(3f/4f)),
                        new Scalar(0, 255, 0), 4);
            }


            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                switchableWebcam.pauseViewport();
            }
            else
            {
                switchableWebcam.resumeViewport();
            }
        }
    }
}