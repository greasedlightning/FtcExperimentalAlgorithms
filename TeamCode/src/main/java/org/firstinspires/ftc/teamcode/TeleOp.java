package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

public class TeleOp extends OpMode {
    ElapsedTime a = new ElapsedTime();
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;

    CRServo spinnyThing;
    CRServo scoop;

    @Override
    public void init() {
        topLeft = hardwareMap.dcMotor.get("topLeft"); //1
        topRight = hardwareMap.dcMotor.get("topRight"); //0
        bottomLeft = hardwareMap.dcMotor.get("bottomLeft"); //2
        bottomRight = hardwareMap.dcMotor.get("bottomRight"); //3

        spinnyThing = hardwareMap.crservo.get("spinnyThing");
        scoop = hardwareMap.crservo.get("scoop");

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinnyThing.setDirection(CRServo.Direction.REVERSE); //CR Servo reverse
        scoop.setDirection(CRServo.Direction.FORWARD);       //CR Servo forward

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        spinnyThing.setPower(0);
        //spinnyThing.MAX_POSITION = 280; //280 degree angular freedom
    }
    public void move(double angle, double rotation){
        double x = Math.sin(angle);
        double y = Math.cos(angle);
        topLeft.setPower(y - x);
        bottomRight.setPower(y - x);
        topRight.setPower(y + x);
        bottomLeft.setPower(y + x);
    }

    @Override
    public void loop() {
        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightY = gamepad1.right_stick_y;
        double rightX = gamepad1.right_stick_x;
        double angle = Math.atan(leftY/leftX);


        if(gamepad1.right_trigger == 1) {
            //move(angle, 0);
            topLeft.setPower     (leftY + leftX - rightX);   //make leftX - if it doesn't work
            topRight.setPower    (leftY - leftX + rightX);
            bottomLeft.setPower  (leftY - leftX - rightX);
            bottomRight.setPower (leftY + leftX + rightX);   //make leftX - if it doesn't work
        } else {
            //Normal Movement
            telemetry.addData("y left thing", leftY);
            telemetry.addData("x left thing", leftX);
            topLeft.setPower(    (leftY + leftX - rightX) * .5);     //make leftX - if it doesn't work
            topRight.setPower(   (leftY - leftX + rightX) * .5);
            bottomLeft.setPower( (leftY - leftX - rightX) * .5);
            bottomRight.setPower((leftY + leftX + rightX) * .5);     //make leftX - if it doesn't work
        }

        spinnyThing.setPower(gamepad2.right_stick_x);

        scoop.setPower(gamepad2.left_stick_y);

    }

}
