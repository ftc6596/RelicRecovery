package org.firstinspires.ftc.teamcode.v1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by RAMSEY on 10/31/2017.
 */
@Autonomous(name="BLUE STONE 21", group ="BLUE")
public class BlueStone21 extends LinearOpMode{
    Robot robot = new Robot();
    String vuMarkID = "UNKNOWN";
    String whichIsLeft = "UNKNOWN";
    double  distance = 0;
    double heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry,true);
        waitForStart();

            //reset encoders
            robot.resetDriveEncoders();
            robot.stopMotors();
            robot.linearLift.setPower(-.5);
            whichIsLeft = robot.whichIsLeft(BlueStone21.this);
            robot.linearLift.setPower(0);
            vuMarkID = robot.vuMark(BlueStone21.this, 1);
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
        if(whichIsLeft.equals("RED")){
            robot.turnWithIMU(BlueStone21.this,0.1,"left",10);
            robot.stopMotors();
            sleep(100);
            robot.jewel.setPosition(1);
            robot.turnWithIMU(BlueStone21.this,0.1,"right",0);
            robot.stopMotors();
            sleep(100);
            robot.imuDrive(BlueStone21.this, 0.5, 21, 0);
        } else if (whichIsLeft.equals("BLUE")) {
//            robot.turnWithIMU(RedStone11.this,0.1,"left",10);
            robot.imuDrive(BlueStone21.this, 0.5, 21, 0);
            robot.stopMotors();
            sleep(100);
            robot.jewel.setPosition(1);
        } else{
            robot.jewel.setPosition(1);
            robot.imuDrive(BlueStone21.this, 0.5, 21, 0);
        }

            robot.stopMotors();
            sleep(100);
            robot.imuDrive(BlueStone21.this,0.5,5,0);
            if (vuMarkID.equals("RIGHT")){
                distance = 20;
                heading = 5;
            } else if (vuMarkID.equals("CENTER")){
                distance = 14;
                heading = 10;
            } else if (vuMarkID.equals("LEFT")){
                distance = 5;
                heading = 5;
            } else {
                distance = 14;
                heading = 5;
            }
            robot.stopMotors();
            sleep(100);
            robot.turnWithIMU(BlueStone21.this,0.2,"right",90);
            robot.stopMotors();
            sleep(100);
            //drive straight to bonus column based off vumark location id
            robot.imuDrive(BlueStone21.this,0.5,distance,-90);
            robot.stopMotors();
            sleep(100);
            robot.turnWithIMU(BlueStone21.this,0.2,"left",heading);
            robot.stopMotors();
            sleep(100);
            robot.scoreBlock(BlueStone21.this, 180);
            stop();
        }
    }
