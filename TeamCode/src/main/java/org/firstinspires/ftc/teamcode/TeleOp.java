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
import org.firstinspires.ftc.teamcode.experimental.CompVision;
import org.firstinspires.ftc.teamcode.experimental.Vector2;

import java.lang.Math;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {

    CompVision cam;

    ElapsedTime time = new ElapsedTime();
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;

    DcMotor flyWheel;
    
    DcMotor armBase;
    CRServo claw;

    @Override
    public void init() {
        // Control Hub
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

        // Expansion Hub
        claw = hardwareMap.crservo.get("claw");                 //0 CR Servo
        flyWheel = hardwareMap.dcMotor.get("flyWheel");         //0
        armBase = hardwareMap.dcMotor.get("armBase");           //1

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyWheel.setDirection(DcMotor.Direction.FORWARD);
        armBase.setDirection(DcMotor.Direction.FORWARD);

        claw.setDirection(CRServo.Direction.FORWARD);

        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armBase.setPower(0);
        claw.setPower(0);
        flyWheel.setPower(0);

        cam = new CompVision(hardwareMap, 1);
    }

    //Read encoders
    public void readEncoder(){
        telemetry.addData("armBase Encoder Ticks: ", armBase.getCurrentPosition());
        cam.printDebugCam1(telemetry);
        telemetry.update();
    }

    public void readEncoderArm(){
        telemetry.addData("Encoder Ticks: ", armBase.getCurrentPosition());
        telemetry.update();
    }

    public void resetClaw(int time) throws InterruptedException {
        claw.setPower(0);
        claw.setPower(-1);
        Thread.sleep(time);
        claw.setPower(0);
    }
    public void closeClaw(int time) throws InterruptedException {
        claw.setPower(0);
        claw.setPower(1);
        Thread.sleep(time);
        claw.setPower(0);
    }

    @Override
    public void loop() {
        cam.printStats(telemetry);
        Vector2[] cubes = cam.getCubes();
        telemetry.addLine("Cubes: " + cubes.length);
        for (int i = 0; i < cubes.length; i++) {
            telemetry.addLine("Cube" + i + ": (" + cubes[i].x + ", " + cubes[i].y + ")");
        }

        if (cubes.length > 0) {
            Vector2 closest = cubes[0];
            Vector2 goal = new Vector2(57, 34);
            Vector2 goal2 = new Vector2(9, 34);
            double pixelToInch = 5 / goal.distTo(goal2);
            double dist = goal.distTo(closest) * pixelToInch;
            double angle = goal.angleTo(closest);
            telemetry.addLine("Dist to: " + dist);
            telemetry.addLine("Angle to: " + angle);
        }

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
        if(gamepad1.right_trigger > 0.6f) {
            //move(angle, 0);
            topLeft.setPower     (leftY - leftX - rightX);
            topRight.setPower    (leftY + leftX + rightX);
            bottomLeft.setPower  (leftY + leftX - rightX);
            bottomRight.setPower (leftY + leftX + rightX);
        } else if (gamepad1.left_trigger > 0.6f) {
            topLeft.setPower(    (leftY - leftX - rightX) * .4);
            topRight.setPower(   (leftY + leftX + rightX) * .4);
            bottomLeft.setPower( (leftY + leftX - rightX) * .4);
            bottomRight.setPower((leftY + leftX + rightX) * .4);
        } else {
            topLeft.setPower(    (leftY - leftX - rightX) * .8);
            topRight.setPower(   (leftY + leftX + rightX) * .8);
            bottomLeft.setPower( (leftY + leftX - rightX) * .8);
            bottomRight.setPower((leftY + leftX + rightX) * .8);
        }

        //flyWheel set to gamepad 2 right bumper
        if(gamepad2.right_bumper){
            flyWheel.setPower(1);
        }
        else if(gamepad2.left_bumper){
            flyWheel.setPower(-1);
        }
        else{
            flyWheel.setPower(0);
        }
        
        //Claw and arm
        double armOffset;
        if(gamepad2.right_trigger > 0){
            armOffset = 0.25;
        }
        else{
            armOffset = 0.5;
        }

        armBase.setPower(-armVal*armOffset);

        if(valcR > 0){
            claw.setPower(valcR*-0.5);
        }
        else if(valcL > 0){
            claw.setPower(valcL*0.5);
        }
        else{
            claw.setPower(0);
        }

    }
}
