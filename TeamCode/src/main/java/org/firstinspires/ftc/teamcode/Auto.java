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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.experimental.CompVision;
import org.firstinspires.ftc.teamcode.experimental.Vector2;

@Disabled
@Autonomous(name = "Auto", group = "autonomous")
public class Auto extends LeoOpMode
{
    private int path = -1;

    public void runAutoPath(int starting) throws InterruptedException {
        initRobo();
        cam = new CompVision(hardwareMap, 1);
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        while(path == -1){
            cam.printDebugCam1(telemetry);
            cam.printStats(telemetry);
            path = cam.getPath();
            telemetry.addLine("Path chosen: " + path);
            telemetry.update();
        }
        telemetry.addLine("Path chosen: " + path);
        telemetry.update();
        sleep(100);


        /*switch (starting) {
            case 0:
                runLeft(1);
                break;
            case 2:
                runLeft(-1);
                break;
            case 1:
                runRight(1);
                break;
            case 3:
                runRight(-1);
                break;
        }*/

        int t, dir;
        double forward = 19.701-2-1-10;
        double angle = 37.5 + 2.5 + 2;
        double ang, add;
        closeClaw();
        Thread.sleep(200);
        moveArm(100, .3);
        linearY(5, .7);
        Thread.sleep(300);
        if (path == 2) {
            t = 670;
            dir = -1;
            ang = -90-angle-7.5;
            add = 0;
        }else if(path == 1){
            t = 730;
            dir = -1;
            ang = -90-angle-7.5;
            add = 0;
        }else{
            dir = 1;
            t = 150;
            ang = angle;
            add = 0;
        }
        //turn to point towards hub
        turnHeading(ang);
        Thread.sleep( 50);
        // go near the hub
        linearY(dir*(forward+add), .3);
        if (path<=1) {
            Thread.sleep(50);
            moveArm(t, .2);
        }
        Thread.sleep(200);
        linearY(dir*5, .2);
        if (path==2){
            Thread.sleep(50);
            moveArm(t, .2);
        }
        Thread.sleep(200);
        resetClaw(300);
        Thread.sleep(50);
        linearY(dir*-(forward+add)/2, .5);
        Thread.sleep(50);

        moveArm(250, .4);
        Thread.sleep(100);
        turnHeading(62);
        Thread.sleep(100);
        linearY(-16, .55);
        linearY(-1.5, .1);
        Thread.sleep(10);

        flyWheel.setPower(.6);
        Thread.sleep(2000);
        flyWheel.setPower(0);

        //linearY(1, .8);
        turnHeading(0, .8);
        Thread.sleep(50);
        linearY(8, .9);
        moveArm(500, .75);


    }

    public void runLeft(int side) throws InterruptedException {
        side = 1;
        int t, dir;
        double ang, add;
        closeClaw();
        linearY(6, .5);
        Thread.sleep(300);


        //10
        //160
        //720

        if (path == 0) {
            //turn to point towards hub
            turnHeading(37.5);
            Thread.sleep(100);
            moveArm(100, .2);
            Thread.sleep(100);
            linearY(7, .2);

            Thread.sleep(100);
            resetClaw(300);
            Thread.sleep(100);

            linearY(-7, .5);
            Thread.sleep(100);
        } else if (path == 1) {
            //turn to point towards hub
            turnHeading(37.5);
            Thread.sleep(100);
            moveArm(220, .2);
            Thread.sleep(100);
            linearY(7, .2);

            Thread.sleep(100);
            resetClaw(300);
            Thread.sleep(100);

            linearY(-7, .5);
            Thread.sleep(100);
        } else {
            turnHeading(-180 + 37.5);
            Thread.sleep(100);
            linearY(-10, .2);

            moveArm(720, .4);
            Thread.sleep(200);

            Thread.sleep(100);
            resetClaw(300);
            Thread.sleep(100);

            linearY(10, .4);
            Thread.sleep(100);
            turnHeading(-37.5);
        }

        turnHeading(-90);
        Thread.sleep(50);
        moveArm(200, .5);
        Thread.sleep(50);

        //linearY(-30, 1);

    }
    public void runRight(int side) throws InterruptedException {
        side = 1;
        int t, dir;
        double forward = 19.701-2-1-10;
        double angle = 37.5;
        double ang, add;
        closeClaw();
        linearY(3, .5);
        Thread.sleep(300);
        if (path == 2) {
            t = 700;
            dir = -1;
            ang = -90-angle-15;
            add = 4;
        }else if(path == 1){
            t = 730;
            dir = -1;
            ang = -90-angle-15;
            add = 3.3;
        }else{
            dir = 1;
            t = 125;
            ang = angle;
            add = 0;
        }
        //turn to point towards hub
        turnHeading(ang*side);
        Thread.sleep(100);
        // go near the hub
        linearY(dir*forward, .4);
        if (path<=1) {
            Thread.sleep(100);
            moveArm(t, .5);
        }
        Thread.sleep(100);
        linearY(dir*5, .2);
        if (path==2){
            Thread.sleep(100);
            moveArm(t, .5);
        }
        Thread.sleep(100);
        resetClaw(300);
        Thread.sleep(100);
        linearY(dir*-(forward+add)/2, .5);
        Thread.sleep(100);
        turnHeading(70*side);
        linearY(-16, .6);
        linearY(-2.5, .1);

        flyWheel.setPower(0.35);
        Thread.sleep(3500);
        flyWheel.setPower(0);

        //go back to base
        linearY(10,.7);
        turnHeading(90);
        closeClaw();
        moveArm(100,.5);
        linearY(60, 1);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        /*initRobo();
        cam = new CompVision(hardwareMap, 1);
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while(path == -1){
            cam.printDebugCam1(telemetry);
            cam.printStats(telemetry);
            path = cam.getPath();
            telemetry.addLine("Path chosen: " + path);
            telemetry.update();
        }

        //cam.changePipeLine(0, 1);
        sleep(100);

        /*linearY(12, 0.2);
        turnHeading(-90, 0.2);
        linearY(5, 0.2);
        turnHeading(20, 0.2);
*/
        //runAutoPath(path);
       // grabCube();

/*
        closeClaw();
        moveArm(200, 0.5);
        sleep(100);
        turnHeading(-10, 0.2);
        linearY(9, 0.2);
        turnHeading(-45, 0.2);
        linearY(4, 0.2);
        turnHeading(-70, 0.2);
        linearY(5, 0.2);

        turnHeading(-90, 0.2);
        linearY(8, 0.4);
        turnHeading(-100);
        linearY(2, 0.4);
        turnHeading(-130);
        resetClaw(300);
        turnHeading(-180);


 */
    }
    public void grabCube() throws InterruptedException {
        //up = left
        //down = right
        //right = far
        //left = close

        resetClaw(300);
        boolean inPosition = false;
        while (!inPosition) {
            Vector2[] cubes = cam.getCubes();
            telemetry.addLine("Cubes: " + cubes.length);
            for (int i = 0; i < cubes.length; i++) {
                telemetry.addLine("Cube" + i + ": (" + cubes[i].x + ", " + cubes[i].y + ")");
            }
            telemetry.update();

            if (cubes.length > 0) {
                Vector2 closest = cubes[0];
                Vector2 goal = new Vector2(57, 34);
                Vector2 goal2 = new Vector2(9, 34);
                double pixelToInch = 5 / goal.distTo(goal2);

                double dist = goal.distTo(closest) * pixelToInch;
                double angle = goal.angleTo(closest) / 6f;
                turnHeading(angle + getAngle(), 0.2);

                if (dist < 5) {
                    linearY(dist, .2);
                    inPosition = true;
                } else {
                    linearY(2, .2);
                }
            } else {
                linearY(4, .4);
            }
        }
        closeClaw();
        sleep(400);
        moveArm(200, 0.5);
        sleep(2000);
    }
}
