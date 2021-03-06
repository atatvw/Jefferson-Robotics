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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Blue Inner Depot Drag", group="Pushbot")

public class PushbotAutoBlueInnerDepotDrag extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // Neverrest 40
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.460;  // was 0.5
    static final double TURN_SPEED = 0.1;  // was 0.5
    static final double LIFT_MAX_SPEED = 0.25; // maximum arm speed
    static final double LEFT_SERVO_CLOSED = 0.65;
    static final double LEFT_SERVO_OPEN = 0.0;
    static final double RIGHT_SERVO_CLOSED = 0.27;
    static final double RIGHT_SERVO_OPEN = 1.0;


static final double LEFT_DOWN = 1.0;
static final double LEFT_UP= 0.15;
static final double RIGHT_DOWN = 0.5;
static final double RIGHT_UP = 0.0;



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Touch Sensor
       robot.rearTouch.setMode(DigitalChannel.Mode.INPUT);

     //   robot.markerServo.setPosition(SERVO_LOW);
        waitForStart();

        // Lower Self
        //delayTime(1.0);
       // robot.liftMotor.setPower(LIFT_MAX_SPEED);
       // delayTime(4.2);
      //  robot.liftMotor.setPower(0.0);

        //Drive to position (approx center of foundation)
        encoderDrivePosition(DRIVE_SPEED, DRIVE_SPEED, 280, 280, 10.0,false); // move out from wall
        encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,-1250,1400,10.0,false); //Turn to line
        encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,6000,6000,10.0,false);//Move to build zone
        encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,-1250,1450,10.0, false);//Turn away from foundation

       // Move to foundation
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftDriveMotor.setPower(-0.25);
        robot.rightDriveMotor.setPower(-0.25);

        while(robot.rearTouch.getState() == true) {
            delayTime(0.01);
        }

        robot.leftDriveMotor.setPower(0);
        robot.rightDriveMotor.setPower(0);

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Latch onto foundation
        robot.leftDragServo.setPosition(LEFT_DOWN);
        robot.rightDragServo.setPosition(RIGHT_DOWN);

        //Turn to move foundation
        encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,-2500,2800,10.0, false);
        encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,1250,-1400,10.0,false);
        encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,6000,6000,10.0, false);
        //drop marker
       // delayTime(0.2);
      //  robot.markerServo.setPosition(SERVO_HIGH);
       // delayTime(0.2);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void timeDrive(double leftPower, double rightPower, double timeOut) {

        if(opModeIsActive()) {

            robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            runtime.reset();
            robot.leftDriveMotor.setPower(leftPower);
            robot.rightDriveMotor.setPower(rightPower);

            while(opModeIsActive() && (runtime.seconds() < timeOut)) {

                telemetry.addData("Step Time: %7d", runtime.seconds());
                telemetry.update();

            }

            robot.leftDriveMotor.setPower(0);
            robot.rightDriveMotor.setPower(0);

        }
    }

    public void delayTime(double timeOut) {

        if(opModeIsActive()) {

           runtime.reset();
            while(opModeIsActive() && (runtime.seconds() < timeOut)) {

                telemetry.addData("Step Time: %7d", runtime.seconds());
                telemetry.update();

            }

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
    public void encoderDriveDistance(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDriveMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDriveMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDriveMotor.setTargetPosition(newLeftTarget);
            robot.rightDriveMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDriveMotor.setPower(Math.abs(speed));
            robot.rightDriveMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDriveMotor.isBusy() && robot.rightDriveMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDriveMotor.getCurrentPosition(),
                        robot.rightDriveMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDriveMotor.setPower(0);
            robot.rightDriveMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderDrivePosition(double ls, double rs,
                                     int lp, int rp,
                                     double timeoutS, boolean reverseMode) {
        int newLeftTarget;
        int newRightTarget;

        double leftSpeed;
        double rightSpeed;
        int leftPosition;
        int rightPosition;

        //Remap values if reverse mode
        if (!reverseMode){
            leftSpeed = ls;
            rightSpeed = rs;
            leftPosition = lp;
            rightPosition = rp;
        }else{
            leftSpeed = -rs;
            rightSpeed =-ls;
            leftPosition = -rp;
            rightPosition =-lp;
        }

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDriveMotor.getCurrentPosition() + leftPosition;
            newRightTarget = robot.rightDriveMotor.getCurrentPosition() + rightPosition;
            robot.leftDriveMotor.setTargetPosition(newLeftTarget);
            robot.rightDriveMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDriveMotor.setPower(leftSpeed);
            robot.rightDriveMotor.setPower(rightSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDriveMotor.isBusy() || robot.rightDriveMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDriveMotor.getCurrentPosition(),
                        robot.rightDriveMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDriveMotor.setPower(0);
            robot.rightDriveMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
