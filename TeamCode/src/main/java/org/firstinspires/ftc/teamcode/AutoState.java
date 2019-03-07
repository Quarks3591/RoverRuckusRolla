package org.firstinspires.ftc.teamcode;

/**
 * Created by User on 10/2/2018.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by User on 11/7/2017.
 */
@Autonomous
public class AutoState extends LinearOpMode {
    //creating hardware objects
    Chassis chassis = new Chassis(true);

    //init input variables
    private String color = "Red";
    private String side = "Left";
    private String target = "Center";

    ElapsedTime runtime = new ElapsedTime();


    @Override public void runOpMode() {
        //init hardware
        chassis.init(hardwareMap);

        //vuforia setupw2
        VuforiaTrackables relicTrackables = this.chassis.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        //side/color input for match
        while (!opModeIsActive()) {
            if(gamepad1.x || gamepad2.x) color = "Blue";
            if (gamepad1.b || gamepad2.b) color = "Red";
            telemetry.addData("Color {Blue (X), Red (B)}", color);

            if(gamepad1.a || gamepad2.a) side = "Left";
            if (gamepad1.y || gamepad2.y) side = "Right";
            telemetry.addData("Side {Left (A), Right (Y)}", side);

            telemetry.update();
        }
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
}
