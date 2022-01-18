package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MiniOp", group = "TeleOp")
public class MiniOpMode extends LinearOpMode {
    DcMotor topLeft;

    public void initRobo() {
        topLeft = hardwareMap.dcMotor.get("bottomRight"); //Insert name ordering to config

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeft.setPower(0);
    }
    public void readEncoder(){
        telemetry.addData("topLeft Encoder Ticks: ", topLeft.getCurrentPosition());
        telemetry.update();
    }
    public void moveRobot(int ticks, double power) throws InterruptedException{
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setTargetPosition(ticks);

        topLeft.setPower(power);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (topLeft.isBusy() && opModeIsActive()){
            readEncoder();
        }

        topLeft.setPower(0);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initRobo();
        waitForStart();
        //moveArm(1000, 1);
    }
}
