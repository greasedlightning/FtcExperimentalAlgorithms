package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class Parent extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();

    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;

    DcMotor arm;

    BNO055IMU imu;
    Orientation lastAngles;
    double globalAngle;
    PIDControl pid;

    public void initRobo(){
        topLeft = hardwareMap.dcMotor.get("topLeft"); //1
        topRight = hardwareMap.dcMotor.get("topRight"); //0
        bottomLeft = hardwareMap.dcMotor.get("bottomLeft"); //2
        bottomRight = hardwareMap.dcMotor.get("bottomRight"); //3

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        //random stuff
        arm = hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //imu.isGyroCalibrated();
        lastAngles = new Orientation();

        //init orientation (old)
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;

        double kP, kI, kD;
        kP = 0.1;
        kI = 1;
        kD = 0.5;
        this.pid = new PIDControl(kP, kI, kD);
        this.pid.setSetpoint(0);
        this.pid.setOutputRange(0, 1);
        this.pid.setInputRange(-90, 90);
        this.pid.enable();
        waitForStart();
    }

    //Gyro
    public void turnHeading(double angle) throws InterruptedException {
        //Tolerances
        double subAng = angle - 0.5;
        double supAng = angle + 0.5;

        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Currently buffering, position :", angles.firstAngle);
            telemetry.update();
            if (angle < angles.firstAngle) {
                topLeft.setPower(-0.05);
                topRight.setPower(0.05);
                bottomLeft.setPower(-0.05);
                bottomRight.setPower(0.05);
                //Within Tolerances
                if (subAng < angles.firstAngle && angles.firstAngle < supAng) {
                    break;
                }
            } else if (angle > angles.firstAngle) {
                topLeft.setPower(0.05);
                topRight.setPower(-0.05);
                bottomLeft.setPower(0.05);
                bottomRight.setPower(-0.05);
                //Within Tolerances
                if (subAng < angles.firstAngle && angles.firstAngle < supAng) {
                    break;
                }
            }
        }
        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
    }

    public void turnHeadingNF(double angle) throws InterruptedException {
        double speed = 0.05;
        int lastDir = 0;
        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Currently buffering position: ", angles.firstAngle);
            telemetry.update();

            double dif = Math.abs(angles.firstAngle - angle);
            if (dif < 0.001) {
                break;
            }

            double pow = speed;
            if (angle < angles.firstAngle) {
                pow *= 1;
                if (lastDir == 0) { lastDir = 1; } else if (lastDir == -1) { speed /= 2.0;  lastDir = 1; }
            } else {
                pow *= -1;
                if (lastDir == 0) { lastDir = -1; } else if (lastDir == 1) { speed /= 2.0; lastDir = -1; }
            }
            topLeft.setPower(-pow );
            topRight.setPower(pow);
            bottomLeft.setPower(-pow);
            bottomRight.setPower(pow);
        }
        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
    }

    ///Set Single Power
    public void setSinglePow(double pow){
        topLeft.setPower(pow);
        topRight.setPower(pow);
        bottomLeft.setPower(pow);
        bottomRight.setPower(pow);
    }

    public void setColumnPow(double powLeft, double powRight){
        topLeft.setPower(powLeft);
        topRight.setPower(powLeft);
        bottomLeft.setPower(powRight);
        bottomRight.setPower(powRight);
    }

    //remember to set setpoint
    public double[] calcCorrection(double power, double angle){
        double powerCorrection = this.pid.performPID(angle);
        double[] corrections = {(power - powerCorrection) ,(power + powerCorrection), power};
        return corrections;
    }

    //Set Each Power
    public void setEachPow(double tLpow, double tRpow, double bLpow, double bRpow){
        topLeft.setPower(tLpow);
        topRight.setPower(tRpow);
        bottomLeft.setPower(bLpow);
        bottomRight.setPower(bRpow);
    }

    public double getAngle(){
        Orientation angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - this.lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        this.globalAngle += deltaAngle;
        this.lastAngles = angles;
        return this.globalAngle;
    }

    public void linear(double pow, int time) throws InterruptedException {
        double [] correction = this.calcCorrection(pow, this.getAngle());
        this.setColumnPow(correction[0], correction[1]);
        Thread.sleep(time);
    }

    //Forward/Backward
    public void linearY(double pow, int time) throws InterruptedException {
        setSinglePow(pow);

        Thread.sleep(time);
    }

    //Sideways
    public void linearX(double pow, int time) throws InterruptedException{
        setEachPow(-pow, pow, -pow, pow);

        Thread.sleep(time);
    }
    //Read encoders
    public void readEncoder() {
        telemetry.addData("topLeft Encoder Ticks: ", topLeft.getCurrentPosition());
        telemetry.addData("topRight Encoder Ticks: ", topRight.getCurrentPosition());
        telemetry.addData("bottomLeft Encoder Ticks: ", bottomLeft.getCurrentPosition());
        telemetry.addData("bottomRight Encoder Ticks: ", bottomRight.getCurrentPosition());
        telemetry.update();
    }
    public void moving(int ticks){
        arm.setTargetPosition(ticks);
        arm.setPower(0.5);
        telemetry.addData("Encoder Ticks: ", arm.getCurrentPosition());
        telemetry.update();
        while (arm.isBusy()) {

        }
        arm.setPower(0);
    }
    /*
    public void moveDis(double distance, boolean dir){ //enter distance in meters
        double avrVelocity = 0.00169;
        long x = Math.round(distance/avrVelocity);
        int milliSeconds = Math.round(x);
        if(dir){
            linearY(05, milliSeconds);
        }
        else{
            linearY(-0.5, milliSeconds);
        }
    }

    ⠀⠀⠀⠀⠀⠀⠀⠀⣠⣤⣤⣤⣤⣤⣶⣦⣤⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⡿⠛⠉⠙⠛⠛⠛⠛⠻⢿⣿⣷⣤⡀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⠋⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⠈⢻⣿⣿⡄⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣸⣿⡏⠀⠀⠀⣠⣶⣾⣿⣿⣿⠿⠿⠿⢿⣿⣿⣿⣄⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣿⣿⠁⠀⠀⢰⣿⣿⣯⠁⠀⠀⠀⠀⠀⠀⠀⠈⠙⢿⣷⡄⠀
⠀⠀⣀⣤⣴⣶⣶⣿⡟⠀⠀⠀⢸⣿⣿⣿⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣷⠀
⠀⢰⣿⡟⠋⠉⣹⣿⡇⠀⠀⠀⠘⣿⣿⣿⣿⣷⣦⣤⣤⣤⣶⣶⣶⣶⣿⣿⣿⠀
⠀⢸⣿⡇⠀⠀⣿⣿⡇⠀⠀⠀⠀⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠃⠀
⠀⣸⣿⡇⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠉⠻⠿⣿⣿⣿⣿⡿⠿⠿⠛⢻⣿⡇⠀⠀
⠀⣿⣿⠁⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣧⠀⠀
⠀⣿⣿⠀⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⠀⠀
⠀⣿⣿⠀⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⠀⠀
⠀⢿⣿⡆⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⡇⠀⠀
⠀⠸⣿⣧⡀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⠃⠀⠀
⠀⠀⠛⢿⣿⣿⣿⣿⣇⠀⠀⠀⠀⠀⣰⣿⣿⣷⣶⣶⣶⣶⠶⢠⣿⣿⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⣿⣿⡇⠀⣽⣿⡏⠁⠀⠀⢸⣿⡇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⣿⣿⡇⠀⢹⣿⡆⠀⠀⠀⣸⣿⠇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢿⣿⣦⣄⣀⣠⣴⣿⣿⠁⠀⠈⠻⣿⣿⣿⣿⡿⠏⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠈⠛⠻⠿⠿⠿⠿⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀

    boolean red_sus = true;
    while(red_sus){
        telemetry.addLine("amogus");
        telemetry.update();
    }

⣿⣿⣿⣿⣿⣿⣿⡿⠿⠛⠛⢉⣉⣉⣉⣉⡛⠛⠛⠛⠛⠛⠛⠻⠻⠿⠿⣿⣿⣿⣿
⣿⣿⣿⣿⠟⠋⠉⣤⣴⣶⣿⣿⣿⣿⣿⣿⣿⣿⣶⣦⣤⠀⠁⠀⠄⠀⠄⠀⠌⠉⠛
⣟⠉⠁⠀⠀⢀⣤⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣄⠀⠐⠀⠐⠀⠐⠀
⡟⠀⠀⠀⠒⠋⠉⠉⠈⠀⠉⠉⠙⠻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⡀⠈⢀⠈⠀⠄
⠀⠀⠀⠀⣀⣴⠖⢁⣠⣶⣶⣶⣦⣴⣿⣿⣻⣾⣿⣾⣿⣻⣿⣿⣿⣧⠀⠀⡀⠂⠀
⠀⣀⣰⣾⡿⠁⢠⣻⣽⣿⣟⣿⣟⣿⣿⣽⣿⢿⣷⡿⣟⣿⣷⣷⠹⣿⡆⠀⠀⡀⠁
⠘⣿⣿⡟⠁⠀⡾⣽⣿⣯⣿⣿⣻⣿⣽⣿⣽⣿⡿⣿⣿⢿⣻⣾⡀⠸⣯⠀⠀⠀⠄
⠀⣺⡿⠀⠀⢨⣟⣿⣽⣯⣿⣾⣿⣿⣽⣿⣽⣿⢿⣟⣿⣿⣿⣿⡿⡆⠈⠀⠀⠂⠀
⠀⢺⡯⠀⠀⣼⢾⢻⠿⠻⠿⠽⣷⣿⣯⣿⠿⠚⠛⠙⠉⢈⣀⠠⠌⣀⠀⡀⠀⢀⠁
⠀⠈⠟⢀⣀⢈⣀⠄⢀⢤⠀⠀⠀⠉⣉⣌⠀⢀⠀⠊⢑⣒⠈⠀⣸⣿⡀⣿⡀⠀⡀
⠀⠀⢰⡆⠀⠈⠁⠂⣩⣤⣤⣤⡀⠀⣾⣿⣦⣼⣿⣾⣿⢿⡾⢀⣿⣟⣿⡯⠂⠀⡀
⠀⠀⠀⠑⠀⠀⠹⣶⣿⣿⣿⣿⠋⢀⣷⣿⣟⣿⡯⢿⢾⣿⣷⣾⣿⣻⣿⠃⠀⢀⠀
⠀⠀⠀⠀⠀⠀⠐⣤⡿⣿⡾⠂⠰⡅⠓⡿⣿⡻⠃⣄⠈⠚⣽⣾⡿⣟⠛⠀⠀⠄⠀
⠀⠄⠀⠀⠀⠀⠀⠀⠙⠁⠀⠀⠀⣤⣤⣀⣀⣴⣿⣿⣷⣤⡀⢸⣿⡇⠀⠀⠐⠈⣿
⠀⠀⠂⠀⠀⠀⠀⠀⢰⡦⠀⠐⠟⠋⠉⠉⣁⣀⣀⠀⠀⠀⠀⣰⣿⡧⠀⠐⠀⢼⣿
⠀⠁⡀⠂⠀⠀⠀⠀⠹⣽⡄⠀⠀⠀⠀⣉⣥⣤⣦⣶⣿⣷⢐⣿⣿⠇⠀⠀⠒⠛⠛
⠀⠄⠀⠐⠈⠀⠀⠀⠀⠛⣮⣄⡦⠀⠀⣄⣻⣿⣿⣿⣻⣾⣿⣿⡛⠀⣄⠀⠈⠀⠂
⠀⠠⠈⠀⠂⠁⡀⠀⠀⠀⠀⠑⠛⠅⠀⣼⣿⣿⣿⣿⣟⣿⠷⠋⢀⣰⣿⡆⠠⣄⠀
⠀⠄⠂⠐⠀⠁⠀⠀⢀⣄⠀⠀⠀⠀⠀⢻⣿⣿⣿⣿⠎⠁⠀⣠⣾⣿⣿⡇⢸⡾⡀
⠀⠠⠐⠀⠂⠁⠀⠀⠸⣾⣗⣦⣀⡀⠀⠀⠛⠿⣿⠏⠀⠤⠚⠛⣿⣿⣿⡃⣸⣿⠆
⠀⠄⠀⠀⠀⠀⠀⣤⠀⢿⣿⣿⠉⠉⠀⠀⠀⠀⠉⠁⠀⠀⠀⠀⢸⣿⣿⢰⣿⣿⢧
⠀⠀⠀⠀⡀⠀⠨⣿⢦⠈⢿⣷⡀⠀⠀⠀⢀⡄⠀⠀⢢⣀⠐⢶⣽⣿⠏⣾⣿⣿⣯
⣿⡳⠀⠀⠀⠀⠨⣿⣯⣷⡈⢻⣿⣿⡏⣴⡟⠀⠀⠀⠈⣿⣷⣦⣻⣿⣿⣿⣿⣿⡷
⣿⠀⠀⠀⠁⠀⠀⣿⣿⣷⣿⣦⣙⣿⣿⣻⠁⠀⣲⠀⠀⠹⠿⠟⠟⠛⠋⠋⢉⣉⡉
⡇⠀⠀⠄⠂⠀⠀⠻⠿⠟⠋⠋⠉⠉⠈⠀⠀⠀⠀⠀⠀⢀⠀⡀⠄⢹⣿⠀⠌⣿⡇
⣷⣄⡀⠠⠀⠀⠀⠀⠀⠀⠀⡀⢀⠀⠄⠀⠄⠂⠐⠈⢀⠠⠀⢀⠠⢸⣿⡅⠀⢿⣿
⣿⣿⣿⣦⣄⡀⢀⠠⠀⠂⠁⠀⡀⠀⠄⠂⢀⠐⠀⠂⠀⡀⠄⢀⠀⠨⣿⣇⠀⢘⣿
⣿⣿⣿⣿⣿⣿⣷⣤⣄⣐⠈⠀⡀⠄⠀⠂⢀⠠⠐⠀⠁⢀⠀⡀⠄⠂⣿⣷⡀⠄⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣷⣶⣤⣤⣥⣀⣄⣀⣂⣈⣀⣠⣠⣤⣤⣿⣿⣾⣿⣿

    boolean its_finger_licking_good = true;
    */

}
