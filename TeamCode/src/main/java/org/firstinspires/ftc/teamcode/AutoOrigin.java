package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
        linearY(0.5, 1938/2);
        turnHeading(90.6358);
        linearY(0.5, 1260/2);
        turnHeading(90.665245);
        linearY(0.5, 1451/2);
        turnHeading(-252.75327);
        linearY(0.5, 1164/2);
        turnHeading(341.9354);
        linearY(0.5, 770/2);

         */
        linearY(.25, 600);
        Thread.sleep(1000);
        turnHeadingNF(80);
        Thread.sleep(500);
        linearY(1, 1400);
    }

}