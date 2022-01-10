package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "encoderTest", group = "autonomous")
public class encoderTest extends ParentOrigin{
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
