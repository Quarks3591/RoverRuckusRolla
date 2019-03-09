package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by User on 10/30/2018.
 */

@TeleOp

public class TeleState extends OpMode
{
    //Initializes all motors and servos
    DcMotor FrontLeft;
    DcMotor BackLeft;
    DcMotor FrontRight;
    DcMotor BackRight;

    DcMotor HangSpool;
    DcMotor HangLift;
    DcMotor BoxLift;
    DcMotor Collector;

    Servo depositor;
    Servo TeamMarker;

    public void init()
    {
        //maps all motors and servos for the phone
        FrontLeft = hardwareMap.dcMotor.get("front_left_drive");
        BackLeft = hardwareMap.dcMotor.get("back_left_drive");
        FrontRight = hardwareMap.dcMotor.get("front_right_drive");
        BackRight = hardwareMap.dcMotor.get("back_right_drive");
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        HangLift = hardwareMap.dcMotor.get("hang_lift");
        HangSpool = hardwareMap.dcMotor.get("hang_spool");
        BoxLift = hardwareMap.dcMotor.get("box_lift");
        Collector = hardwareMap.dcMotor.get("collector");

        depositor = hardwareMap.servo.get("depositor");
        TeamMarker = hardwareMap.servo.get("team_marker");

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HangLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HangSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BoxLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop()
    {
        //move forwards
        if (gamepad1.right_trigger > 0.1)
        {
            FrontLeft.setPower(gamepad1.right_trigger);
            BackLeft.setPower(gamepad1.right_trigger);
            FrontRight.setPower(gamepad1.right_trigger);
            BackRight.setPower(gamepad1.right_trigger);
        }
        //move backwards
        else if (gamepad1.left_trigger > 0.1)
        {
            FrontLeft.setPower(-gamepad1.left_trigger);
            BackLeft.setPower(-gamepad1.left_trigger);
            FrontRight.setPower(-gamepad1.left_trigger);
            BackRight.setPower(-gamepad1.left_trigger);
        }
        //move right
        else if (gamepad1.dpad_right)
        {
            FrontLeft.setPower(1.0);
            BackLeft.setPower(-1.0);
            FrontRight.setPower(-1.0);
            BackRight.setPower(1.0);
        }
        //move left
        else if (gamepad1.dpad_left)
        {
            FrontLeft.setPower(-1.0);
            BackLeft.setPower(1.0);
            FrontRight.setPower(1.0);
            BackRight.setPower(-1.0);
        }
        //rotation
        else if (gamepad1.left_stick_x != 0)
        {
            FrontLeft.setPower(gamepad1.left_stick_x);
            BackLeft.setPower(gamepad1.left_stick_x);
            FrontRight.setPower(-gamepad1.left_stick_x);
            BackRight.setPower(-gamepad1.left_stick_x);
        }
        else
        {
            BackLeft.setPower(0.0);
            FrontRight.setPower(0.0);
            FrontLeft.setPower(0.0);
            BackRight.setPower(0.0);
        }

        //raise hang lift
        if (gamepad2.y)
        {
            HangLift.setPower(0.8);
            HangSpool.setPower(.4);
        }
        //lower hang lift
        if (gamepad2.x)
        {
            HangLift.setPower(-0.8);
            HangSpool.setPower(-.4);
        }
        else
        {
            HangLift.setPower(0.0);
            HangSpool.setPower(0.0);
        }

        //run collector
        if (gamepad2.right_trigger > 0.1)
        {
            Collector.setPower(gamepad2.right_trigger);
        }
        if (gamepad2.left_trigger > 0)
        {
            Collector.setPower(-gamepad2.left_trigger);
        }
        else
        {
            Collector.setPower(0.0);
        }

        //raise box lift
        if (gamepad2.dpad_up)
        {
            BoxLift.setPower(1.0);
        }
        //lower box lift
        if (gamepad2.dpad_down)
        {
            BoxLift.setPower(-1.0);
        }
        else
        {
            BoxLift.setPower(0.0);
        }

        //flip box with depositor servo
        if (gamepad2.a){
            depositor.setPosition(1.0);
        }
        if (gamepad2.b){
            depositor.setPosition(0.0);
        }
    }
}
