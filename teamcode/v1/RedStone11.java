package org.firstinspires.ftc.teamcode.v1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by RAMSEY on 10/31/2017.
 */
@Autonomous(name="RED STONE 11", group ="RED")
public class RedStone11 extends LinearOpMode{
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
        //Find X value of Red Ball and compare to Blue Ball X value
        robot.linearLift.setPower(-.5);
        whichIsLeft = robot.whichIsLeft(RedStone11.this);
        robot.linearLift.setPower(0);
        vuMarkID = robot.vuMark(RedStone11.this, 1);
        robot.openArms();
        //lower lift
        robot.linearLift.setPower(.5);
        sleep(1000);
        robot.linearLift.setPower(0);
        robot.closeArms();
        sleep(200);
        //raise lift
        robot.linearLift.setPower(-.75);
        sleep(2250);
        robot.linearLift.setPower(0);

        //extend jewel servo between balls
        robot.jewel.setPosition(0);
        sleep(500);
        //Knock BLUE ball off
        if(whichIsLeft.equals("RED")){
            robot.turnWithIMU(RedStone11.this,0.1,"right",10);
            robot.stopMotors();
            sleep(100);
            robot.jewel.setPosition(1);
            robot.turnWithIMU(RedStone11.this,0.1,"left",0);
            robot.stopMotors();
            sleep(100);
            robot.imuDrive(RedStone11.this, -0.5, -21, 0);
            robot.stopMotors();
            sleep(100);
        } else if (whichIsLeft.equals("BLUE")) {
//            robot.turnWithIMU(RedStone11.this,0.1,"left",10);
            robot.imuDrive(RedStone11.this, -0.5, -7, 0);
            robot.jewel.setPosition(1);
            robot.imuDrive(RedStone11.this, -0.5, -14, 0);
            robot.stopMotors();
            sleep(100);
        } else{
            //servo jewel arm back to home
            robot.jewel.setPosition(1);
            sleep(100);
            robot.imuDrive(RedStone11.this, -0.5, -21, 0);
            robot.stopMotors();
            sleep(100);
        }
        //based on vumark set distance to drive as below
        if (vuMarkID.equals("RIGHT")){
            distance = 4.5;
            heading = 95;
        } else if (vuMarkID.equals("CENTER")){
            distance = 12;
            heading = 90;
        } else if (vuMarkID.equals("LEFT")){
            distance = 19.5;
            heading = 85;
        } else {
            distance = 12;
            heading = 90;
        }
        //drive straight to bonus column based off vumark location id
        robot.imuDrive(RedStone11.this,-0.5,-distance,0.0);
        robot.stopMotors();
        sleep(100);
        //turn back of robot towards wall to prepare for glyph scoring
        robot.turnWithIMU(RedStone11.this,0.2,"left",heading);
        robot.stopMotors();
        sleep(100);
        robot.scoreBlock(RedStone11.this,90);
        stop();
    }
}
