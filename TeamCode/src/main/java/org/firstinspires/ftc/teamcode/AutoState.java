package org.firstinspires.ftc.teamcode;

/**
 * Created by User on 10/2/2018.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by User on 11/7/2017.
 */
@Autonomous
public class AutoState extends LinearOpMode {
    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.75;
    static final double     TURN_SPEED              = 0.5;

    //creating hardware objects
    Chassis chassis = new Chassis(true);

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AcRQ+zL/////AAABmY1C1gY/ZkQvj7F1OvmMC/srTdyIgHwBg4eKi" +
            "a3ks1CRw0/8p6QcRy8Z5Fn95CZX2t3VOVzsDYUtcXMDdeRMh38BZClV8VXhy4xOc231eQ+DzLvHbCJIHWhFgJHpB" +
            "1B5tcv4xHwXUpBOKpa0VwdAgsS2/Sde5sBx02hyUcPaDx9llJH2GoaYrhNOPh6F6g2xXckTHz9XZuXKDwXTps0q+" +
            "5YRoBKWf6kADm5T4ynt4umaiHJ0UzmDk7mSZKjF/Tzd1gIbQD3z0mnUqTs0RIIC/4XCOLMvIqFYjx2+KspopYRET/" +
            "D2gzJHnZ1Mi/GTY9CPBCWR/H6MnudAC0fI8Iuv29aZMQfg7KNWHCdIT42RvWzO";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        //Reset all Encoders
        chassis.stopAndReset();

        //Set to run using encoders
        chassis.runUsingEncoders();

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

            while (opModeIsActive()) {
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
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        //init hardware
        chassis.init(hardwareMap);

        //vuforia setupw2
        VuforiaTrackables relicTrackables = this.chassis.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        waitForStart();

        //lower to the ground and let go of bracket
        chassis.extendLift();
        sleep(1000);
        chassis.stopLift();
        sleep(100);
        chassis.strafeLeftDistance(1.0, 1000);
        chassis.retractLift();
        sleep(1000);
        chassis.stopLift();
        sleep(100);

        //scan minerals using phone


        //knock gold mineral
        boolean left = false;
        boolean middle = false;
        boolean right = false;
        chassis.driveForwardDistance(1.0, 1000);


        //move to team depot and drop team marker
        if (left = true){
            chassis.turnToPosition(-45);
            chassis.driveForwardDistance(1.0, 5000);
        }
        else if (middle = true){
            chassis.driveForwardDistance(1.0, 5000);
        }
        else if (right = true){
            chassis.turnToPosition(45);
            chassis.driveForwardDistance(1.0, 5000);
        }
        chassis.markerHolder.setPosition(120);
        sleep(500);
        chassis.markerHolder.setPosition(0);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

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
