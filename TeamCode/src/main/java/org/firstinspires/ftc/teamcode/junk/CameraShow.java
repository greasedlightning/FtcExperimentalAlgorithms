/*
 * Copyright (c) 2019 OpenFTC Team
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

package org.firstinspires.ftc.teamcode.junk;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LeoOpMode;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
 */
@Disabled
@TeleOp(name = "CameraShow", group = "teleop")
public class CameraShow extends LeoOpMode
{
    OpenCvCamera phoneCam;
    private int path = -1;
    private float d0;
    private float d1;
    private float d2;

    private float c0;
    private float c1;
    private float c2;

    private boolean armOn = false;
    public int ticks = 0;


    public void armOn(int ticks){
        this.ticks = ticks;
    }
    public void armOff(){
        ticks = -1;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initRobo();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(new SamplePipeline());

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        /*
         * Send some stats to the telemetry
         */
        while(path >= -1){
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addLine("d0: " + d0);
            telemetry.addLine("d1: " + d1);
            telemetry.addLine("d2: " + d2);

            telemetry.addLine("Path chosen: " + path);
            telemetry.update();
        }
        phoneCam.stopStreaming();

    }
    public void runAutoPath(int path) throws InterruptedException{
        //                turnHeadingNF(35);
        //
        //            }
        //            else{
        //                turnHeadingNF(-125);
        //            }
        //move forward by a little to be able to turn - SAME FOR ALL PATHS
        linearY(.25, 450);
        Thread.sleep(300);
        turnHeadingNF(-125);
        int t, dir;
        if (path==2){
            t = 660;
            dir = -1;

            //forward code fast
            Thread.sleep(100);
            linearY(dir*.25, 600);
            Thread.sleep(100);

            //move the arm up to prepare for dropping
            moveArm(t, .45);

            //go inside the hub
            Thread.sleep(10);
            linearY(dir*.1, 375);
            Thread.sleep(100);

            //open the claw (drop freight)
            resetClaw(200);
            Thread.sleep(100);

            //backward code (slow, then fast)
            linearY(dir*-.25, 150);
            Thread.sleep(100);
            linearY(dir*-.125, 400);
            Thread.sleep(100);

            //little back (dunno why?)
            linearY(dir*-.10, 300);

            //close the claw back
            closeClaw(300);
            Thread.sleep(50);

            //put the arm back down
            moveArm(5, .1);
            Thread.sleep(1000);

        }else if (path==1){

        }else{

        }
    }


    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused = false;

        @Override
        public Mat processFrame(Mat input)
        {


            //320, 240

           // if (path == -1) {
                d0 = greenDensityRatio(input, 140, 0, 70, 70);
                d1 = greenDensityRatio(input, 110, 130, 80, 90);
                d2 = greenDensityRatio(input, 90, 290, 80, 40);
                /*
                d0 = greenDensityRatio(input, 50, 0, 120, 100);
                d1 = greenDensityRatio(input, 50, 100, 120, 100);
                d2 = greenDensityRatio(input, 50, 200, 120, 100);
*/
                if (d0 > d1) {
                    if (d0 > d2) {
                        path = 0;
                    } else {
                        path = 2;
                    }
                } else {
                    if (d1 > d2) {
                        path = 1;
                    } else {
                        path = 2;
                    }
                }
            //}


            int c = 255;
            if (path == 0) {
                c = 0;
            }
            Imgproc.rectangle(input,
                    new Point(150, 0),
                    new Point(150 + 50, 0 +50),
                    new Scalar(c, 0, 0), 2
            );
            c = 255;
            if (path == 1) {
                c = 0;
            }
            Imgproc.rectangle(input,
                    new Point(120, 140),
                    new Point(120 + 60, 150 + 40),
                    new Scalar(0, c, 0), 2
            );
            c = 255;
            if (path == 2) {
                c = 0;
            }
            Imgproc.rectangle(input,
                    new Point(100, 300),
                    new Point(160, 300 + 20),
                    new Scalar(0, 0, c), 2
            );

            return input;
        }


        //may make a ratio one, but this works well for now
        private float greenDensity(Mat input, int _y, int _x, int height, int width) {
            float green = 0;
            int tot = 0;
            for (int x = _x; x < _x + width && x < input.height(); x++) {
                for (int y = _y; y < _y + height && y < input.width(); y++) {
                    double[] d = input.get(x, y);
                    green += d[1];
                    tot++;
                }
            }
            return green / tot;
        }

        private float greenDensityRatio(Mat input, int _y, int _x, int height, int width) {
            float r = 0;
            float g = 0;
            float b = 0;
            int tot = 0;
            for (int x = _x; x < _x + width && x < input.height(); x++) {
                for (int y = _y; y < _y + height && y < input.width(); y++) {
                    double[] d = input.get(x, y);
                    r += d[0];
                    g += d[1];
                    b += d[2];
                    tot++;
                }
            }
            return g / tot - r / tot - b / tot;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                phoneCam.pauseViewport();
            }
            else
            {
                phoneCam.resumeViewport();
            }
        }
    }
}
