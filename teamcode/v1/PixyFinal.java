package org.firstinspires.ftc.teamcode.v1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import static java.lang.Thread.sleep;

/**
 * Created by RAMSEY on 11/10/2017.
 */
@Autonomous(name="PixyFinal", group ="PIXY")

public class PixyFinal extends LinearOpMode {
    I2cDeviceSynch pixyCam;

    double x, y, width, height, numObjects;

    byte[] redBall;
    byte[] blueBall;

    @Override
    public void runOpMode() throws InterruptedException {

        pixyCam = hardwareMap.i2cDeviceSynch.get("pixy");
        double redBallX;
        double blueBallX;
        byte[] redBall;
        byte[] blueBall;
        int i = 0;
        int redBallLeftCount = 0;
        int blueBallLeftCount = 0;
        waitForStart();
        pixyCam.engage();

        while(opModeIsActive()){

            redBall = pixyCam.read(0x51,5);
            redBallX = Math.abs(redBall[1]);
            blueBall = pixyCam.read(0x52,5);
            blueBallX = Math.abs(blueBall[1]);
            if (redBallX < blueBallX){
                redBallLeftCount++;
            } else if (redBallX > blueBallX){
                blueBallLeftCount++;
            }

            telemetry.addData("RED X VALUE NOT BYTE", redBallX);
            telemetry.addData("BLUE X VALUE NOT BYTE", blueBallX);
            telemetry.addData("RED X VALUE", 0xff&redBall[1]);
            telemetry.addData("BLUE X VALUE", 0xff&blueBall[1]);
            telemetry.addData("RED COUNT",redBallLeftCount);
            telemetry.addData("BLUE COUNT", blueBallLeftCount);
            telemetry.update();
            sleep (500);
        }

    }
}