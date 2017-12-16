package org.firstinspires.ftc.teamcode.v1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by RAMSEY on 10/31/2017.
 */
@Autonomous(name="BLUE STONE 11", group ="BLUE")
public class BlueStone11 extends LinearOpMode{
    Robot robot = new Robot();
    String vuMarkID = "UNKNOWN";
    String whichIsLeft = "UNKNOWN";
    double distance = 0;
    double heading = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry,true);

        waitForStart();
        //reset encoders
        robot.resetDriveEncoders();
        robot.stopMotors();
        robot.linearLift.setPower(-.5);
        whichIsLeft = robot.whichIsLeft(BlueStone11.this);
        robot.linearLift.setPower(0);
        vuMarkID = robot.vuMark(BlueStone11.this, 1);
        robot.openArms();
        robot.linearLift.setPower(.5);
        sleep(1000);
        robot.linearLift.setPower(0);
        robot.closeArms();
        sleep(200);
        robot.linearLift.setPower(-.75);
        sleep(2250);
        robot.linearLift.setPower(0);
        //extend jewel servo between balls
        robot.jewel.setPosition(0);
        sleep(500);
        //Knock RED ball off
        if(whichIsLeft.equals("BLUE")){
            robot.turnWithIMU(BlueStone11.this,0.1,"left",10);
            robot.stopMotors();
            sleep(100);
            robot.jewel.setPosition(1);
            robot.turnWithIMU(BlueStone11.this,0.1,"right",0);
            robot.stopMotors();
            sleep(100);
            robot.imuDrive(BlueStone11.this, 0.5, 21, 0);
        } else if (whichIsLeft.equals("RED")) {
//            robot.turnWithIMU(RedStone11.this,0.1,"left",10);
            robot.imuDrive(BlueStone11.this, 0.5, 21, 0);
            robot.stopMotors();
            sleep(100);
            robot.jewel.setPosition(1);
        } else{
            robot.jewel.setPosition(1);
            robot.imuDrive(BlueStone11.this, 0.5, 21, 0);
        }
        robot.stopMotors();
        sleep(100);
        if (vuMarkID.equals("RIGHT")){
            distance = 22;
            heading = 88;
        } else if (vuMarkID.equals("CENTER")){
            distance = 15;
            heading = 90;
        } else if (vuMarkID.equals("LEFT")){
            distance = 8;
            heading = 90;
        } else {
            distance = 15;
            heading = 90;
        }
        //drive straight to bonus column based off vumark location id
        robot.imuDrive(BlueStone11.this,0.5,distance,0.0);
        robot.stopMotors();
        sleep(100);
        //turn back of robot towards wall to prepare for glyph scoring
        robot.turnWithIMU(BlueStone11.this,0.2,"left",heading);
        robot.stopMotors();
        sleep(100);
        robot.scoreBlock(BlueStone11.this, 90);
        stop();
    }
}
