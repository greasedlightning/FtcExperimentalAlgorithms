package org.firstinspires.ftc.teamcode.junk;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.LeoOpMode;

@Disabled
@Autonomous(name = "encoderTest", group = "autonomous")
public class encoderTest extends LeoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobo();
        //moveRobot(200, .1);
        /*while (true) {
            moveArm(200, .1);
        }*/
        linearY(-.2, 500);
        Thread.sleep(10000);
        linearY(-.2, 750);
        Thread.sleep(10000);
        linearY(-.2, 1000);
        Thread.sleep(10000);
        linearY(-.2, 1250);
        Thread.sleep(10000);
        linearY(-.2, 1500);
        Thread.sleep(10000);
    }
}
