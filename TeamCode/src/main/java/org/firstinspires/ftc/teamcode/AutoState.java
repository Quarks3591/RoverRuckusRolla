package org.firstinspires.ftc.teamcode;

/**
 * Created by User on 10/2/2018.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * Created by User on 11/7/2017.
 */
@Autonomous
public class AutoState extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.75;
    static final double     TURN_SPEED              = 0.5;

    //Init booleans for the Gold particle's location
    boolean left = false;
    boolean center = false;
    boolean right = false;

    //creating hardware objects
    Chassis chassis = new Chassis(true); //Initialize our Pushbot

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AV24vGn/////AAABmWE3UiJNLEVUhnXBR0kFSAdSCm/ONqDkdhfldODUiAgt7kKezyX6YzsSJKhTw9oQ6ymcvGtNBuOK84/OuemOmGv8XYoYgTkJUu8pDIRl6ExvjNifxnPTeZO3RTzAkruuJ8NMNpn0MeM+aPBXJx3URftKMoEvV4uyVNAE4A+WTvQbDk8t7RXRWrKZRI9SkJsphRK9EJCGRYnpHM7NfF2uOT7V7UjqNCuedIyff2HfeWqVHaXqOCWiavel8ASAdqDqaFkARsy+VcCGU6KFR/Xn1YcBZj9dTV5Rzub51OrlW+FO/2HXGbUWRNV4e7PNfTvsyrRFCcrnXG0vk15HyrzZ5MaPOBcBTAXEGqsQWTpkXJiW";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        //init hardware
        chassis.init(hardwareMap);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            //while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    left = true;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    right = true;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    center = true;
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            //}
            //Reset all Encoders
            chassis.stopAndReset();

            //Set to run using encoders
            chassis.runUsingEncoders();

            //lower to the ground and let go of bracket
            chassis.extendLift();
            sleep(1000);
            chassis.stopLift();
            sleep(100);
            encoderDrive(1.0, -15, 15, 15, -15, 5);
            chassis.retractLift();
            sleep(100);
            chassis.stopLift();
            sleep(100);


            //knock gold mineral
            if (left = true){
                if (center = false) {
                    if (right = false) {
                        chassis.turnToPosition(-15);
                        encoderDrive(1.0, -38, -38, -38, -38, 10);
                        chassis.turnToPosition(15);
                        encoderDrive(1.0, -24, -24, -24, -24, 8);
                    }
                }
            }
            else if (center = true){
                if (left = false) {
                    if (right = false) {
                        encoderDrive(1.0, -88, -88, -88, -88, 15);
                    }
                }
            }
            else if (right = true){
                if (left = false) {
                    if (center = false) {
                        chassis.turnToPosition(15);
                        encoderDrive(1.0, -38, -38, -38, -38, 10);
                        chassis.turnToPosition(15);
                        encoderDrive(1.0, -24, -24, -24, -24, 8);
                    }
                }
            }
            chassis.TeamMarker.setPosition(0.0);
            sleep(500);
            chassis.TeamMarker.setPosition(.5);
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double frontLeftInches, double backLeftInches, double frontRightInches, double backRightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = chassis.BackLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = chassis.BackRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = chassis.FrontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = chassis.FrontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            chassis.BackLeft.setTargetPosition(newBackLeftTarget);
            chassis.BackRight.setTargetPosition(newBackRightTarget);
            chassis.FrontLeft.setTargetPosition(newFrontLeftTarget);
            chassis.FrontRight.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            chassis.runToPosition();

            // reset the timeout time and start motion.
            runtime.reset();
            chassis.driveForward(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.allMotorsAreBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        chassis.FrontLeft.getCurrentPosition(),
                        chassis.FrontRight.getCurrentPosition(),
                        chassis.BackLeft.getCurrentPosition(),
                        chassis.BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            chassis.stop();

            // Turn off RUN_TO_POSITION
            chassis.runUsingEncoders();

            //  sleep(250);   // optional pause after each move
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}

