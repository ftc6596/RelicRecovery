package org.firstinspires.ftc.teamcode.v1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by RAMSEY on 12/14/2017.
 */
@Autonomous(name="ScoreBlockTest", group ="TEST")
public class ScoreBlockTest  extends LinearOpMode {
    Robot robot = new Robot();
    String vuMarkID = "UNKNOWN";
    String whichIsLeft = "UNKNOWN";
    double distance = 0;
    double heading = 0;

//    @Override
   public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry, true);
        robot.closeArms();
        waitForStart();
        robot.turnWithIMU(ScoreBlockTest.this,.2,"left",90);
        robot.scoreBlock(ScoreBlockTest.this,90);
//        robot.getMoreBlocks(ScoreBlockTest.this);
        stop();
    }
}
