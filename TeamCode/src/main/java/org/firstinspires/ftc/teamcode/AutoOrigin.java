package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "AutonomousOrigin", group = "autonomous")
public class AutoOrigin extends ParentOrigin{
    @Override
    public void runOpMode() throws InterruptedException {
        initRobo();

        /*turnHeading(-0.8756714);
        linearY(0.5, 1938);
        turnHeading(90.6358);
        linearY(0.5, 1260);
        turnHeading(90.665245);
        linearY(0.5, 1451);
        turnHeading(-252.75327);
        linearY(0.5, 1164);
        turnHeading(341.9354);
        linearY(0.5, 770);
*/
        /*
        turnHeading(-0.8756714);
        readEncoder();
        linearY(0.5, 1938/2);
        readEncoder();
        turnHeading(90.6358);
        readEncoder();
        linearY(0.5, 1260/2);
        readEncoder();
        turnHeading(90.665245);
        readEncoder();
        linearY(0.5, 1451/2);
        readEncoder();
        turnHeading(-252.75327);
        readEncoder();
        linearY(0.5, 1164/2);
        readEncoder();
        turnHeading(341.9354);
        readEncoder();
        linearY(0.5, 770/2);
        readEncoder();
         */
        for (int x=0; x<30; x++) {
            topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            topRight.setTargetPosition(x);

            topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topRight.setPower(.25);
            while (topRight.isBusy()) {

            }
            topRight.setPower(0);
            topRight.setTargetPosition(0);
            topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Thread.sleep(500);
        }

    }

}