package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class ParentOrigin extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();

    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;

    CRServo spinnyThing;
    CRServo scoop;

    BNO055IMU imu;

    // experimental PID control - localization
    Orientation lastAngles;
    double globalAngle;
    PIDControl pid;

    public void initRobo(){
        topLeft = hardwareMap.dcMotor.get("topLeft"); //1
        topRight = hardwareMap.dcMotor.get("topRight"); //0
        bottomLeft = hardwareMap.dcMotor.get("bottomLeft"); //2
        bottomRight = hardwareMap.dcMotor.get("bottomRight"); //3

        spinnyThing = hardwareMap.crservo.get("spinnyThing"); //Servo 0
        scoop = hardwareMap.crservo.get("scoop");
        scoop.setDirection(CRServo.Direction.FORWARD);

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

        spinnyThing.setDirection(CRServo.Direction.REVERSE); //Reverse servo

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

        spinnyThing.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // experimental PID control - setup --------------------------
        //init orientation (old)
        lastAngles = new Orientation();
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;

        double kP, kI, kD; // constant parameters
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
        double subAng = angle - 2;
        double supAng = angle + 2;

        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("topLeft Encoder Ticks: ", topLeft.getCurrentPosition());
            telemetry.addData("topRight Encoder Ticks: ", topRight.getCurrentPosition());
            telemetry.addData("bottomLeft Encoder Ticks: ", bottomLeft.getCurrentPosition());
            telemetry.addData("bottomRight Encoder Ticks: ", bottomRight.getCurrentPosition());
            telemetry.update();
            if (angle < angles.firstAngle) {
                topLeft.setPower(-0.05);
                topRight.setPower(0.05);
                bottomLeft.setPower(-0.05);
                bottomRight.setPower(0.05);
                if (subAng < angles.firstAngle && angles.firstAngle < supAng) {
                    break;
                }
            }
            else if (angle > angles.firstAngle) {
                topLeft.setPower(0.05);
                topRight.setPower(-0.05);
                bottomLeft.setPower(0.05);
                bottomRight.setPower(-0.05);
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

    //experimental code with PID control ----------------------------
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
    //remember to set setpoint
    public double[] calcCorrection(double power, double angle){
        double powerCorrection = this.pid.performPID(angle);
        double[] corrections = {(power - powerCorrection) ,(power + powerCorrection), power};
        return corrections;
    }
    public void setColumnPow(double powLeft, double powRight){
        topLeft.setPower(powLeft);
        topRight.setPower(powLeft);
        bottomLeft.setPower(powRight);
        bottomRight.setPower(powRight);
        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void turnHeadingAM(double angle) throws InterruptedException { // this is not really turn heading, it's weirder
        this.pid.setSetpoint(angle);
        double pow = .4, tol = 1;
        double [] correction; // = new double[2];
        double err = Math.abs(this.globalAngle - angle);
        while (opModeIsActive() && err > tol) {
            telemetry.addData("Currently buffering absolute angle: ", this.getAngle());
            telemetry.update();
            correction = this.calcCorrection(pow, this.getAngle()); // calc correction
            this.setColumnPow(correction[0], correction[1]); // correct
            err = Math.abs(this.globalAngle - angle);
        }
        // reset
        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
    }

    // end experimental ------------------------

    public void turnHeadingNF(double angle) throws InterruptedException {
        double speed = 0.02;
        int lastDir = 0;

        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("Currently buffering position: ", angles.firstAngle);
            telemetry.update();

            double dif = Math.abs(angles.firstAngle - angle);
            if (dif < 1) {
                break;
            }

            double pow = speed;
            if (angle < angles.firstAngle) {
                pow *= 1;
                if (lastDir == 0) { lastDir = 1; }
                else if (lastDir == -1) {
                    speed /= 2.0;  lastDir = 1;
                }
            } else {
                pow *= -1;
                if (lastDir == 0) {
                    lastDir = -1;
                } else if (lastDir == 1) {
                    speed /= 2.0; lastDir = -1;
                }
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

    //Set Each Power
    public void setEachPow(double tLpow, double tRpow, double bLpow, double bRpow){
        topLeft.setPower(tLpow);
        topRight.setPower(tRpow);
        bottomLeft.setPower(bLpow);
        bottomRight.setPower(bRpow);
    }

    //Forward/Backward
    public void linearY(double pow, int time) throws InterruptedException {
        setSinglePow(pow);
        readEncoder();

        Thread.sleep(time);

        setSinglePow(0);
    }

    //Sideways
    public void linearX(double pow, int time) throws InterruptedException {
        setEachPow(-pow, pow, pow, -pow);
        readEncoder();

        Thread.sleep(time);

        setSinglePow(0);
    }

    //Read encoders
    public void readEncoder(){
        telemetry.addData("topLeft Encoder Ticks: ", topLeft.getCurrentPosition());
        telemetry.addData("topRight Encoder Ticks: ", topRight.getCurrentPosition());
        telemetry.addData("bottomLeft Encoder Ticks: ", bottomLeft.getCurrentPosition());
        telemetry.addData("bottomRight Encoder Ticks: ", bottomRight.getCurrentPosition());
        telemetry.update();
    }
    public void rotation(double pow, int time) throws InterruptedException{
        setEachPow(pow, -pow, pow, -pow);
        readEncoder();

        Thread.sleep(time);

        setSinglePow(0);
    }

    //Servo movement
    public void servoMove(double pow, int time) throws InterruptedException {
        spinnyThing.setPower(pow);

        Thread.sleep(time);
    }

    public void moveScoop(double pow, int time) throws InterruptedException {
        scoop.setPower(pow);

        Thread.sleep(time);

        scoop.setPower(0);

    }


}
