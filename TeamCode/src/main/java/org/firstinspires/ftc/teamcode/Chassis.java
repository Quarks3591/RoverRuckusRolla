package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static java.lang.Double.isNaN;

/**
 * Created by User on 10/17/2017.
 */

public class Chassis {

    //create drive train objects
    DcMotor FrontLeft;
    DcMotor BackLeft;
    DcMotor FrontRight;
    DcMotor BackRight;

    DcMotor HangSpool;
    DcMotor HangLift;
    DcMotor BoxLift;
    DcMotor Collector;

    Servo depositor;
    Servo markerHolder;

    boolean vision=false;

    //init other objects
    HardwareMap hwm;

    VuforiaLocalizer vuforia;

    DistanceSensor distance;

    ColorSensor color;

    //set final variables
    final static double INIT = 0.0;
    final static double UP = 0.4;

    //constructors
    Chassis() {}
    Chassis(boolean v) {vision=v;}

    //hardware init method
    public void init(HardwareMap inhwm) {
        hwm = inhwm;

        //connect to hardware
        FrontLeft = hwm.get(DcMotor.class, "front_left_drive");
        BackLeft = hwm.get(DcMotor.class, "back_left_drive");
        FrontRight = hwm.get(DcMotor.class, "front_right_drive");
        BackRight = hwm.get(DcMotor.class, "back_right_drive");

        HangLift = hwm.get(DcMotor.class, "hang_lift");
        HangSpool = hwm.get(DcMotor.class, "hang_spool");
        BoxLift = hwm.get(DcMotor.class, "box_lift");
        Collector = hwm.get(DcMotor.class, "collector");

        depositor = hwm.get(Servo.class, "depositor");
        markerHolder = hwm.get(Servo.class, "markerHolder");

        //set drivetrain direction
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        //set lifts directions
        HangLift.setDirection(DcMotor.Direction.FORWARD);
        HangSpool.setDirection(DcMotor.Direction.FORWARD);
        BoxLift.setDirection(DcMotor.Direction.FORWARD);

        //set collector direction
        Collector.setDirection((DcMotor.Direction.FORWARD));

        //set init power
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
        HangLift.setPower(0);
        HangSpool.setPower(0);
        BoxLift.setPower(0);
        Collector.setPower(0);

        //set init mode
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HangLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HangSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BoxLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set init position
        depositor.setPosition(INIT);
        markerHolder.setPosition(INIT);

        //setup vision if desired
        if(vision) {
            int cameraMonitorViewId = hwm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwm.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = "AXdOcVz/////AAAAGSXgHubMS0r6roxUdjf9DWKA7GTut2LGpmgusuOdBmgcr9vnOOQAc9l3bXYlX+3nFwmePFZh1Brz4BsbF1h6zhAXHx5VvmWWpVeNoCLbxDnyaPBtmZ3k2ZgHLFLTjS2ST/arbrmCSAVUTX9xOgenNw+pcCuZYKQxr34uWWppyhJPPIPP152Gud24gReY/Sg8hM20JYH49E1nRLpIjYA1FTxWy135xOu5SCns1p48AgTLxyy51v0WRBALpIWAW/Qe30gGHb9W3swgPlBj1EVMFgLdCXgp5NAlKNvID6T6G8NaN2dYY64Cv601JDKkcGHM1KNU5N4vn6spJkE0tZI2NqfiSaBKT59+nPsbv5AGQ0IA";

            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        }
    }
    //fixes out-of-range issue with distance sensor
    public double getDistance() {
        double d = distance.getDistance(DistanceUnit.CM);
        if(isNaN(d)) d = 100.0;
        return d;
    }
    //stops chassis
    public void stop() {
        driveForward(0.0);
    }
    //power-Drive Methods
    public void driveForward(double power) {
        FrontLeft.setPower(power);
        BackLeft.setPower(power);
        FrontRight.setPower(power);
        BackRight.setPower(power);
    }
    public void driveBackward(double power) { driveForward(-power);}
    public void turnLeft(double power) {
        FrontLeft.setPower(-power);
        BackLeft.setPower(-power);
        FrontRight.setPower(power);
        BackRight.setPower(power);
    }
    public void turnRight(double power) {
        turnLeft(-power);
    }
    public void strafeLeft(double power) {
        FrontLeft.setPower(power);
        BackLeft.setPower(-power);
        FrontRight.setPower(-power);
        BackRight.setPower(power);
    }
    public void strafeRight(double power) { strafeLeft(-power);}
    //drive Forward/backward to given position
    public void driveToPosition(int position) {
        warmUp();
        setTargetPosition(position);
        execute();
    }
    //set positions for all motors for drivetrain
    public void setTargetPosition(int position) {
        FrontLeft.setTargetPosition(position);
        BackLeft.setTargetPosition(position);
        FrontRight.setTargetPosition(position);
        BackRight.setTargetPosition(position);
    }
    public void setTargetTurn(int position) {
        FrontLeft.setTargetPosition(position);
        BackLeft.setTargetPosition(position);
        FrontRight.setTargetPosition(-position);
        BackRight.setTargetPosition(-position);
    }
    public void setTargetStrafe(int position) {
        FrontLeft.setTargetPosition(-position);
        BackLeft.setTargetPosition(position);
        FrontRight.setTargetPosition(position);
        BackRight.setTargetPosition(-position);
    }
    //check if motors are busy
    public boolean motorsAreBusy() {
        return (FrontLeft.isBusy() || BackLeft.isBusy() || FrontRight.isBusy() || BackRight.isBusy());
    }
    //turn to given position
    public void turnToPosition(int position) {
        warmUp();
        setTargetTurn(position);
        execute();
    }
    //encoder set-up methods for chassis
    public void warmUp() {
        stopAndReset();
        runToPosition();
    }
    public void stopAndReset() {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runToPosition() {
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void execute() {
        driveForward(0.5);
        while(motorsAreBusy()) ;
        stop();
    }
    //strafe to given position
    public void strafeToPosition(int position) {
        warmUp();
        setTargetStrafe(position);
        execute();
    }
    //lift system
    public void extendLift() {
        HangLift.setPower(1.0);
        HangSpool.setPower(1.0);
    }
    public void retractLift() {
        HangLift.setPower(-1.0);
        HangSpool.setPower(-1.0);
    }
    public void stopLift() {
        HangLift.setPower(0.0);
        HangSpool.setPower(0.0);
    }

    //drive a distance method
    public void driveForwardDistance(double power, int distance){
        //reset encoders
        stopAndReset();

        //Set target position
        FrontLeft.setTargetPosition(distance);
        BackLeft.setTargetPosition(distance);
        FrontRight.setTargetPosition(distance);
        BackRight.setTargetPosition(distance);

        //Set to RUN_TO_POSITION mode
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set drive power
        driveForward(power);

        while(FrontLeft.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && BackRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stop();
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeLeftDistance(double power, int distance) {
        //reset encoders
        stopAndReset();

        //Set target position
        FrontLeft.setTargetPosition(distance);
        BackLeft.setTargetPosition(-distance);
        FrontRight.setTargetPosition(-distance);
        BackRight.setTargetPosition(distance);

        //Set to RUN_TO_POSITION mode
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set drive power
        driveForward(power);

        while(FrontLeft.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && BackRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stop();
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeRightDistance(double power, int distance){
        //reset encoders
        stopAndReset();

        //Set target position
        FrontLeft.setTargetPosition(-distance);
        BackLeft.setTargetPosition(distance);
        FrontRight.setTargetPosition(distance);
        BackRight.setTargetPosition(-distance);

        //Set to RUN_TO_POSITION mode
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set drive power
        driveForward(power);

        while(FrontLeft.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && BackRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stop();
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void findColor (){

    }
}