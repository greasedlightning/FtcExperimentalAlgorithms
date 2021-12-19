package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

import java.lang.Math;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
// owo
public class TeleOp extends OpMode {
    ElapsedTime a = new ElapsedTime();
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;

    DcMotor flyWheel;
    
    DcMotor armBase;
    CRServo claw;

    @Override
    public void init() {
        //Control Hub
        topLeft = hardwareMap.dcMotor.get("topLeft");           //1
        topRight = hardwareMap.dcMotor.get("topRight");         //0 
        bottomLeft = hardwareMap.dcMotor.get("bottomLeft");     //2
        bottomRight = hardwareMap.dcMotor.get("bottomRight");   //3

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        //Expansion Hub
        claw = hardwareMap.crservo.get("claw");                 //0 CR Servo
        flyWheel = hardwareMap.dcMotor.get("flyWheel");         //0
        armBase = hardwareMap.dcMotor.get("armBase");           //1

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyWheel.setDirection(DcMotor.Direction.FORWARD);
        armBase.setDirection(DcMotor.Direction.REVERSE);

        claw.setDirection(CRServo.Direction.FORWARD);

        armBase.setPower(0);
        claw.setPower(0);
        flyWheel.setPower(0);
    }

    //Movement functions
    public void move(double angle, double rotation){
        double x = Math.sin(angle);
        double y = Math.cos(angle);
        topLeft.setPower(y - x);
        bottomRight.setPower(y - x);
        topRight.setPower(y + x);
        bottomLeft.setPower(y + x);
    }
    public void moveArm(double angle){
        double x = Math.sin(angle);
        double y = Math.cos(angle);
        armBase.setPower(y-x);
    }

    //Read encoders
    public void readEncoder(){
        telemetry.addData("topLeft Encoder Ticks: ", topLeft.getCurrentPosition());
        telemetry.addData("topRight Encoder Ticks: ", topRight.getCurrentPosition());
        telemetry.addData("bottomLeft Encoder Ticks: ", bottomLeft.getCurrentPosition());
        telemetry.addData("bottomRight Encoder Ticks: ", bottomRight.getCurrentPosition());
        telemetry.addData("armBase Encoder Ticks: ", armBase.getCurrentPosition());
        telemetry.update();
    }

    //double accel = 0.03;
    double decel = 0.9;

    double vTopLeft = 0;
    double vTopRight = 0;
    double vBottomLeft = 0;
    double vBottomRight = 0;

    double pow = 0.5;


    @Override
    public void loop() {
        readEncoder();
        //Gamepad 1 Declarations
        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightY = gamepad1.right_stick_y;
        double rightX = gamepad1.right_stick_x;

        //Gamepad 2 Declarations
        double armVal = gamepad2.right_stick_y;
        double valcR = gamepad2.right_trigger;
        double valcL = gamepad2.left_trigger;

        double angle = Math.atan(leftY/leftX);
        double temp = Math.sqrt(leftY * leftY + leftX * leftX);

        /*
        if (gamepad1.x) {
            pow = 1.0;
        }
        if (gamepad1.a) {
            pow = 0.5;
        }
        if (gamepad1.b) {
            pow = 0.25;
        }


        pow = 0.5 + (gamepad1.right_trigger/2.0);

        topLeft.setPower(    (leftY + leftX - rightX) * pow);
        topRight.setPower(   (leftY - leftX + rightX) * pow);
        bottomLeft.setPower( (leftY - leftX - rightX) * pow);
        bottomRight.setPower((leftY + leftX + rightX) * pow);

         */

        /*
        //Mecanum Wheel Drive//
        if(gamepad1.right_trigger == 1) {
            //move(angle, 0);
            topLeft.setPower     (leftY + leftX - rightX);           //make leftX - if it doesn't work
            topRight.setPower    (leftY - leftX + rightX);
            bottomLeft.setPower  (leftY - leftX - rightX);
            bottomRight.setPower (leftY + leftX + rightX);           //make leftX - if it doesn't work
        } else {
            //Normal Movement
            topLeft.setPower(    (leftY + leftX - rightX) * .5);     //make leftX - if it doesn't work
            topRight.setPower(   (leftY - leftX + rightX) * .5);
            bottomLeft.setPower( (leftY - leftX - rightX) * .5);
            bottomRight.setPower((leftY + leftX + rightX) * .5);     //make leftX - if it doesn't work
        }
        */

        //Traditional Wheel Drive//
        if(gamepad1.right_trigger == 1) {
            //move(angle, 0);
            topLeft.setPower     (leftY - leftX - rightX);
            topRight.setPower    (leftY + leftX + rightX);
            bottomLeft.setPower  (leftY + leftX - rightX);
            bottomRight.setPower (leftY + leftX + rightX);
        }
        else {
            //Normal Movement
            topLeft.setPower(    (leftY - leftX - rightX) * .5);
            topRight.setPower(   (leftY + leftX + rightX) * .5);
            bottomLeft.setPower( (leftY + leftX - rightX) * .5);
            bottomRight.setPower((leftY + leftX + rightX) * .5);
        }


        //flyWheel set to gamepad 1 right bumper
        if(gamepad1.right_bumper){
            flyWheel.setPower(1);
        }
        else if(gamepad1.left_bumper){
            flyWheel.setPower(-1);
        }
        else{
            flyWheel.setPower(0);
        }

        //Claw and arm
        armBase.setPower(armVal*.5);

        if(valcR > 0){
            claw.setPower(valcR*0.5);
        }
        else if(valcL > 0){
            claw.setPower(valcL*-0.5);
        }
        else{
            claw.setPower(0);
        }

    }
}
