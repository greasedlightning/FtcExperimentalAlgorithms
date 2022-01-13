package org.firstinspires.ftc.teamcode.junk;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// overall, the code I edited was a 1.5/10.

@Disabled
public class OdometryCordPos implements Runnable {
    // This assumes  that there are two separate motors for the x and y direction.
    // We might need to use the encoders from the wheels we already have to measure x and y displacement.
    private final DcMotor xMotor, yMotor;
    // Uhh... the criteria for what should be private or not is haphazard...
    double globalXPos = 0, globalYPos = 0, globalAngle = 0;
    double prevX, prevY;
    // set to true value (maybe setup in a file?)
    private final int xInchesPerTick=1, yInchesPerTick=1;
    BNO055IMU gyro;
    //Orientation lastAngles = new Orientation();
    private boolean isRunning=true;

    public OdometryCordPos(DcMotor x, DcMotor y) throws InterruptedException{
        xMotor = x;
        yMotor = y;
        //You can calculate the angle exactly without the need of a gyro with the x and y motors.
        //The angles may become wildly off (error accumulation) very fast
        // This is assumed to be correct...
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.initialize(parameters);
        while (!gyro.isGyroCalibrated()); // what does this do?
        // make sure to calibrate gyro and check the range (since it could be -180 to 180)
    }
    private void odomUpdate(){
        // .getCurrentPosition() returns "ticks" made
        double XPosDelta = xMotor.getCurrentPosition() * xInchesPerTick;
        double YPosDelta = yMotor.getCurrentPosition() * yInchesPerTick;
        globalXPos = prevX + XPosDelta;
        globalYPos = prevY + YPosDelta;
        globalAngle = getAngle();
    }
    private double getAngle(){
        // ZYX? Change this and see what happens.
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // this is unnecessary, just make the current angle equal to the angle read
        // right?
        //double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        //lastAngles = angles;
        //globalAngle += deltaAngle;
        // By the way, you can also get orientation (gl) matrix if so desired
        return (angles.firstAngle + 2*Math.PI) % (2*Math.PI); // normalize
    }

    public void stop(){isRunning=false;}
    public void run(){
        long sleepTime = 1;
        while(isRunning){
            odomUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

