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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.experimental.CompVision;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Auto", group = "autonomous")
public class Auto extends LeoOpMode
{
    CompVision cam;

    private int path = -1;

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
        cam = new CompVision(hardwareMap);
        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

            /*
             * Send some stats to the telemetry
             */
        while(path == -1){
            cam.printDebug(telemetry);
            cam.printStats(telemetry);
            path = cam.getPath();
            telemetry.addLine("Path chosen: " + path);
            telemetry.update();
        }
        cam.stop();
        sleep(100);


        //runAutoPath(path);
        turnHeading(90);

        /*
        //Carousel
        turnHeadingNF(65);
        Thread.sleep(300);
        linearY(-.8, 800);
        Thread.sleep(100);
        startFlywheel(1, 1000);
        Thread.sleep(300);
        linearY(1, 1700);
         */
    }
    public void runAutoPath(int path) throws InterruptedException {
        //                turnHeadingNF(35);
        //
        //            }
        //            else{
        //                turnHeadingNF(-125);
        //            }
        //move forward by a little to be able to turn - SAME FOR ALL PATHS

        int t, dir;
        double forward = 19.701-2-1;
        double angle = 37.5;
        double ang;
        closeClaw();
        Thread.sleep(100);
        linearY(3, .5);
        Thread.sleep(300);
        if (path == 0) {
            t = 680;
            dir = -1;
            ang = -90-angle;
        }else if(path == 1){
            t = 600;
            dir = -1;
            ang = -90-angle;
        }else{
            dir = 1;
            t = 100;
            ang = angle;
        }
        turnHeading(ang);
        Thread.sleep(100);
        linearY(dir*forward, .4);
        Thread.sleep(100);
        moveArm(t, .5);
        Thread.sleep(100);
        linearY(3, .2);
        Thread.sleep(100);
        resetClaw(300);
        Thread.sleep(100);
        linearY(forward/2, .5);
        Thread.sleep(100);
        turnHeading(-90);
        Thread.sleep(50);
        moveArm(200, .5);

    }
}
