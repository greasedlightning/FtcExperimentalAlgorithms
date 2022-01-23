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
        //while(true){
        //    readEncoder();
        //}
        //add code here
        //moveClaw(1);
        Thread.sleep(100);
        //runAutoPath(path);

    }
    public void runAutoPath(int path) throws InterruptedException {
        //move forward by a little to be able to turn - SAME FOR ALL PATHS
        closeClaw();
        linearY(3, .5);
        Thread.sleep(100);
        int t, dir;
        double angle = 37.5;
        double forward = 19.7-2-1;

        if (path == 2) {
            t = 670;
            dir = -1;
            //turn
            double ang = (path>2)? (dir*(90+angle)) : angle;

            turnHeading(ang);

            //forward code fast
            Thread.sleep(100);
            linearY(dir*forward, .5);
            Thread.sleep(100);

            //move the arm up to prepare for dropping
            moveArm(t, .5);

            //go inside the hub

            linearY(dir * .1, 150 + 150);
            Thread.sleep(100);

            //open the claw (drop freight)
            resetClaw(200);
            Thread.sleep(100);

            //forward code fast
            Thread.sleep(100);
            linearY(forward/2, .5);
            Thread.sleep(100);

            //reset robot
            //close the claw back
            turnHeading(-90);

            Thread.sleep(100);
            closeClaw();
            Thread.sleep(100);
            resetClaw(0);

            //put the arm back down
            moveArm(100, .2);
            Thread.sleep(1000);

        } else if (path == 1) {
            t = 600;
            dir = -1;

            //turn
            turnHeadingNF(-125);

            //forward code fast
            Thread.sleep(100);
            linearY(dir * .25, 475);
            Thread.sleep(100);

            //move the arm up to prepare for dropping
            moveArm(t, .45);

            //go inside the hub
            Thread.sleep(10);
            linearY(dir * .1, 375 + 150);
            Thread.sleep(100);

            //open the claw (drop freight)
            resetClaw(200);
            Thread.sleep(100);

            //backward code (slow, then fast)
            linearY(dir * -.25, 150);
            Thread.sleep(100);
            linearY(dir * -.125, 400);
            Thread.sleep(100);

            //little back (dunno why?)
            linearY(dir * -.10, 300);

            //close the claw back
            closeClaw();
            Thread.sleep(50);

            //put the arm back down
            moveArm(5, .1);
            Thread.sleep(1000);

            /*
            linearY(dir*-.25, 150);
            Thread.sleep(100);
            turnHeadingNF(-165); // neg = right
            Thread.sleep(100);
            linearY(dir*-.5, 700); // full towards carousel
             */
        } else {

            t = 100;
            dir = 1;

            //turn
            turnHeadingNF(35);

            //forward code fast
            Thread.sleep(100);
            linearY(dir * .25, 475);
            Thread.sleep(100);

            //move the arm up to prepare for dropping
            moveArm(t, .45);

            //go inside the hub
            Thread.sleep(10);
            linearY(dir * .1, 375 + 150);
            Thread.sleep(100);

            //open the claw (drop freight)
            resetClaw(200);
            Thread.sleep(100);

            //backward code (slow, then fast)
            linearY(dir * -.25, 150);
            Thread.sleep(100);
            linearY(dir * -.125, 400);
            Thread.sleep(100);

            //little back (dunno why?)
            linearY(dir * -.10, 300);

            //close the claw back
            closeClaw();
            Thread.sleep(50);

            //put the arm back down
            moveArm(5, .1);
            Thread.sleep(1000);
        }
    }
}
