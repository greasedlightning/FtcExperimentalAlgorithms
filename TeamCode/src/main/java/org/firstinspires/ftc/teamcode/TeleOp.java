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
    }

    public void readEncoder(){
        telemetry.addData("armBase Encoder Ticks: ", armBase.getCurrentPosition());
        telemetry.update();
    }

    public void setDrivePow(double pow) {
        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightY = gamepad1.right_stick_y;
        double rightX = gamepad1.right_stick_x;

        topLeft.setPower(    (leftY - leftX - rightX) * pow);
        topRight.setPower(   (leftY + leftX + rightX) * pow);
        bottomLeft.setPower( (leftY + leftX - rightX) * pow);
        bottomRight.setPower((leftY + leftX + rightX) * pow);
    }

    @Override
    public void loop() {
        readEncoder();

        //GamePad One - Drive Train
        if      (gamepad1.right_trigger > 0.6f) { setDrivePow(1);   } //Right
        else if (gamepad1.left_trigger > 0.6f)  { setDrivePow(0.2); } //Left
        else                                    { setDrivePow(0.5); } //Normal

        //GamePad Two - Arm
        armBase.setPower(-gamepad2.right_stick_y * 0.25);

        //GamePad Two - Fly Wheel
        if      (gamepad2.right_bumper) { flyWheel.setPower(1);  }
        else if (gamepad2.left_bumper)  { flyWheel.setPower(-1); }
        else                            { flyWheel.setPower(0);  }

        //GamePad Two - Claw
        double close = gamepad2.right_trigger;
        double open = gamepad2.left_trigger;

        if      (close > 0) { claw.setPower(close * -0.5); } //close
        else if (open > 0)  { claw.setPower(open *   0.5); } //open
        else                { claw.setPower(0);            } //stay
    }
}
