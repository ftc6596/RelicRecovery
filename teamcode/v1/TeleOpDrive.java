/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.v1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Hardware class to define the devices on the robot.
 * All device access is managed through the Hardware class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Telop", group="RAMBOT")
//@Disabled
public class TeleOpDrive extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot           = new Robot();              // Use a K9'shardware
//    double          leftServo     = robot.ARM_HOME;                   // Servo safe position
    final double    LEFT_SERVO_SPEED      = 0.1;                            // sets rate to move servo
    final double    RIGHT_SERVO_SPEED       = 0.1 ;                            // sets rate to move servo

    @Override
    public void runOpMode() {
        double pwr = .75;
        double spin = 0;
        double left;
        double right;
        double verticalLift;
        double verticalDrop;
        double takeIn;
        double takeOut;
        double spinRight;
        double spinLeft;
        int rbump = 0;
        int lbump = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap,telemetry,false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        robot.runWithoutEncoders();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //======================GAMEPAD 1 CONTROLS======================

            /// code from previous year to increase/decrease power
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

            if(gamepad1.dpad_up){
                right=.35;
                left=.35;
            }else if(gamepad1.dpad_down){
                right=-.5;
                left=-.5;
            } else {
                left = -gamepad1.left_stick_y;
                right = -gamepad1.right_stick_y;
            }

            left = (left * pwr);
            right = (right * pwr);

            if (gamepad1.right_bumper) {
                if(rbump == 0 && pwr < .75) {
                    pwr = pwr + .25;
                }

                rbump++;
            }else{
                rbump = 0;
            }
            if (gamepad1.left_bumper) {
                if(lbump == 0 && pwr > .25) {
                    pwr = pwr - .25;
                }

                lbump++;
            }else{
                lbump = 0;
            }

            robot.leftFrontDrive.setPower(left);
            robot.leftRearDrive.setPower(left);
            robot.rightFrontDrive.setPower(right);
            robot.rightRearDrive.setPower(right);

            // Move both servos to new position.
//            armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
//            robot.arm.setPosition(armPosition);

//            ======================GAMEPAD 2 CONTROLS======================

            double linearLiftCountHigh = 9579;
            double linearLiftCountLow = 0;

            verticalLift = gamepad2.right_stick_y;

            robot.linearLift.setPower(verticalLift);


            double rightPos = robot.rightArm.getPosition();
            double leftPos = robot.leftArm.getPosition();

            if(gamepad2.left_bumper && robot.leftArm.getPosition() < .20){
                robot.rightArm.setPosition(rightPos+.05);
                robot.leftArm.setPosition(leftPos+.03);
            }
            if(gamepad2.right_bumper && robot.rightArm.getPosition() > .70){
                robot.rightArm.setPosition(rightPos-.03);
                robot.leftArm.setPosition(leftPos-.05);
            }

            if (gamepad2.b){
                robot.rightArm.setPosition(.9);
                robot.leftArm.setPosition(.05);
            }
            if(gamepad2.a){
                robot.rightArm.setPosition(.775);
                robot.leftArm.setPosition(.175);
            }
            if (gamepad2.x){
                robot.rightArm.setPosition(0.70);
                robot.leftArm.setPosition(0.25);
            }
//            if (gamepad2.y){
//                robot.rightArm.setPosition(.70);
//                robot.leftArm.setPosition(.25);
//                robot.rightSpin.setPower(0.8);
//                robot.leftSpin.setPower(-0.8);
//            }
            if(gamepad2.dpad_up ) {
                robot.rightSpin.setPower(-0.8);
                robot.leftSpin.setPower(0.8);
            }else if(gamepad2.dpad_down){
                robot.rightSpin.setPower(0.8);
                robot.leftSpin.setPower(-0.8);
            }else if(gamepad2.dpad_right){
                robot.rightSpin.setPower(0.8);
                robot.leftSpin.setPower(0.8);
            }else if(gamepad2.dpad_left){
                robot.rightSpin.setPower(-0.8);
                robot.leftSpin.setPower(-0.8);
            } else if(gamepad2.y) {
                robot.rightSpin.setPower(0);
                robot.leftSpin.setPower(0);
            }




            // Send telemetry message to signify robot running;
//            telemetry.addData("arm",   "%.2f", armPosition);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("spin", spin);
            telemetry.addData("dpad up",gamepad2.dpad_up);
            telemetry.addData("liftpower",verticalLift);
            telemetry.addData("Right Pos", robot.rightArm.getPosition());
            telemetry.addData("Left Pos", robot.leftArm.getPosition());
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}

// tableLeft = gamepad2.left_trigger;
//////            tableRight = gamepad2.right_trigger;
//////            armValue = gamepad2.right_stick_y;
//////            gripperValue = gamepad2.left_stick_y;
////
////            double turnCounts4Table = 1120;
////
//////            if (robot.turnTable.getCurrentPosition()<= turnCounts4Table && robot.turnTable.getCurrentPosition()>= -turnCounts4Table) {
//////                if ((tableLeft > 0 && tableRight > 0) || (tableLeft == 0 && tableRight == 0)) {
//////                    robot.turnTable.setPower(0);
//////                } else if (tableRight > 0) {
//////                    robot.turnTable.setPower(-tableRight);
//////                } else if (tableLeft > 0) {
//////                    robot.turnTable.setPower(tableLeft);
//////                }
//////            } if(robot.turnTable.getCurrentPosition()>turnCounts4Table){
//////                if(tableRight>0){
//////                    robot.turnTable.setPower(0);
//////                }
//////                }else if (robot.turnTable.getCurrentPosition()<-turnCounts4Table){
//////                if(tableLeft>0){
//////                    robot.turnTable.setPower(0);
//////                }
////
//////            robot.arm.setPower(armValue*.2);
//////            if(armValue > 0 || armValue < 0 ) {
//////                if ((robot.arm.getCurrentPosition() <= 0 && armValue < 0) || (robot.arm.getCurrentPosition() >= 840 && armValue > 0)) {
//////                    robot.arm.setPower(0);
//////                } else {
//////                    robot.arm.setPower(-armValue);
//////                }
//////            }

