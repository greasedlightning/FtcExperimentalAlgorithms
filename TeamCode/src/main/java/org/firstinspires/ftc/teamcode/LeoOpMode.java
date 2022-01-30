package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.experimental.CompVision;

public abstract class LeoOpMode extends LinearOpMode{
    //Motors
    private DcMotor topLeft;
    private DcMotor topRight;
    private DcMotor bottomLeft;
    private DcMotor bottomRight;
    protected DcMotor flyWheel;
    private DcMotor armBase;
    private CRServo claw;

    //Gyro
    private BNO055IMU imu;
    //Cam
    protected CompVision cam;

    //Returning
    private boolean isRecording = false;
    private float x = 0;
    private float y = 0;

    public void initRobo(){
        topLeft = hardwareMap.dcMotor.get("topLeft");           //1 Motor
        topRight = hardwareMap.dcMotor.get("topRight");         //0 Motor
        bottomLeft = hardwareMap.dcMotor.get("bottomLeft");     //2 Motor
        bottomRight = hardwareMap.dcMotor.get("bottomRight");   //3 Motor
        flyWheel = hardwareMap.dcMotor.get("flyWheel");         //0 Motor
        armBase = hardwareMap.dcMotor.get("armBase");           //1 Motor
        claw = hardwareMap.crservo.get("claw");                 //1 CR Servo

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.FORWARD);
        armBase.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(CRServo.Direction.FORWARD);

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        setDrivePow(0);
        setDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel.setPower(0);

        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armBase.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        waitForStart();
    }

    public void linearY(double inches, double power) throws InterruptedException {
        if (isRecording) { addToPath(inches); }
        final double RADIUS=2;
        final double CIRCUMFERENCE = Math.PI*RADIUS*RADIUS;
        final int TPR = 1120;
        int ticks = (int)(inches/CIRCUMFERENCE*TPR);
        moveRobot(ticks, ticks, power);
    }
    public void moveRobot(int left, int right, double power) throws InterruptedException{
        setDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        setDriveTarget(-left, -right);
        setDrivePow(power);
        setDrive(DcMotor.RunMode.RUN_TO_POSITION);
        while(topLeft.isBusy() || topRight.isBusy() || bottomLeft.isBusy() || bottomRight.isBusy()) {}
        setDrivePow(0);
        setDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setDriveTarget(int left, int right){
        topLeft.setTargetPosition(left);
        bottomLeft.setTargetPosition(left);
        topRight.setTargetPosition(right);
        bottomRight.setTargetPosition(right);
    }

    public void turnHeading(double angle) throws InterruptedException {
        turnHeading(angle,0.4f);
    }
    public void turnHeading(double angle, double pow) throws InterruptedException {
        double power = pow;
        double m_P = 5;
        double tol = 1;
        double err = (angle-this.getAngle());
        while(opModeIsActive() && Math.abs(err)>tol){
            int ticks = (int)(m_P*err);
            moveRobot(-ticks, ticks, power);
            err = angle-getAngle();
        }
    }

    public void moveArm(int ticks, double power) throws InterruptedException{
        armBase.setTargetPosition(ticks);
        armBase.setPower(power);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetClaw(int time) throws InterruptedException {
        claw.setPower(0);
        claw.setPower(-1);
        Thread.sleep(time);
        claw.setPower(0);
    }
    public void closeClaw() throws InterruptedException {
        claw.setPower(1);
    }

    public void setPathStart() {
        isRecording = true;
        x = 0;
        y = 0;
    }
    public void reversePath() throws InterruptedException {
        if (!isRecording) return;

        double dist = Math.sqrt(x*x + y*y);
        double angle = Math.atan2(y, x)*180/Math.PI;
        if (angle > 180) { angle = 360 - angle; }
        angle *= -1;

        turnHeading(angle);
        sleep(200);
        linearY(dist, 0.4);
        x = 0;
        y = 0;
        isRecording = false;
    }
    public void addToPath(double inches) {
        double angle = getAngle();
        if (angle < 0) {
            angle = 360 + angle;
        }
        angle = 360 - angle;
        //angle += 90;
        //if (angle > 360) { angle -= 360; }

        x += Math.cos(angle) * inches;
        y += Math.sin(angle) * inches;
    }


    public void setDrive(DcMotor.RunMode mode) {
        topLeft.setMode(mode);
        topRight.setMode(mode);
        bottomLeft.setMode(mode);
        bottomRight.setMode(mode);
    }
    public void setDrivePow(double pow) {
        topLeft.setPower(pow);
        topRight.setPower(pow);
        bottomLeft.setPower(pow);
        bottomRight.setPower(pow);
    }

    public double getAngle(){
        Orientation angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void readEncoder(){
        telemetry.addData("topLeft Encoder Ticks: ", topLeft.getCurrentPosition());
        telemetry.addData("topRight Encoder Ticks: ", topRight.getCurrentPosition());
        telemetry.addData("bottomLeft Encoder Ticks: ", bottomLeft.getCurrentPosition());
        telemetry.addData("bottomRight Encoder Ticks: ", bottomRight.getCurrentPosition());
        telemetry.update();
    }
    public void readEncoderArm(){
        telemetry.addData("Encoder Ticks: ", armBase.getCurrentPosition());
        telemetry.update();
    }
}
