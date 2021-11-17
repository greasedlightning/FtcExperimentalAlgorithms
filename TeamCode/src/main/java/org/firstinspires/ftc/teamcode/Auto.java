package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "Autonomous", group = "autonomous")
public class Auto extends Parent{
    @Override
    public void runOpMode() throws InterruptedException {
        initRobo();
        linear(0.5, 1000);
        turnHeading(-90);
        linear(1, 1000);

    }
}
