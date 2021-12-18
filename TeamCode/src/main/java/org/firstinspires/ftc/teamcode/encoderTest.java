package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "encoderTest", group = "autonomous")
public class encoderTest extends Parent {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobo();
        moving(5000);





    }
}
