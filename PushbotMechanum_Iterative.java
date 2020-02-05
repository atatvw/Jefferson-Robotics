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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *.
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Mechanum Tank", group="Pushbot")

public class PushbotMechanum_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbotMechanum robot       = new HardwarePushbotMechanum(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.\
    static final double UPDATE_TIME = 0.25; // seconds between power updates
    static final double MAX_ACCEL = 0.2 ; // maximum power change per updateTime
    static final double MAX_SPEED = 0.75; // maximum drive speed

    double          targetSpeedRightFront = 0.0;
    double          targetSpeedLeftFront = 0.0;
    double          currentSpeedLeftFront = 0.0;
    double          currentSpeedRightFront = 0.0;
    double          targetSpeedRightRear = 0.0;
    double          targetSpeedLeftRear = 0.0;
    double          currentSpeedLeftRear = 0.0;
    double          currentSpeedRightRear = 0.0;

    boolean         reverseMode = false;
    boolean         bButtonHeld = false;
    boolean         yButtonHeld = false;
    boolean         aButtonHeld = false;
    boolean         xButtonHeld = false;
    boolean         slowMode = false;

    double          driveSpeed = MAX_SPEED;

    //current run time since last update
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.LBDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RBDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LFDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RFDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Possibly add a delay
        robot.LBDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RBDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LFDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RFDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
        currentSpeedRightFront = 0.0;
        currentSpeedLeftFront = 0.0;
        currentSpeedRightRear = 0.0;
        currentSpeedLeftRear = 0.0;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftFront = 0;
        double rightFront = 0;
        double leftRear = 0;
        double rightRear = 0;

        double xSpeed;
        double ySpeed;
        double rSpeed;


        // Run wheels (note: The joystick goes negative when pushed forwards, so negate it)
        if(!reverseMode)
        {
            xSpeed = - gamepad1.left_stick_x;
            ySpeed = - gamepad1.left_stick_y;
             rSpeed =  gamepad1.right_stick_x;
        }
        else
        {
            xSpeed =  gamepad1.left_stick_x;
            ySpeed =  gamepad1.left_stick_y;
            rSpeed =  -gamepad1.right_stick_x;
        }

        //Move Right
        if(xSpeed >= 0.1)
        {
            leftFront = -xSpeed *driveSpeed;
            leftRear = xSpeed*driveSpeed;
            rightFront = xSpeed*driveSpeed;
            rightRear = -xSpeed*driveSpeed;
        }

        //Move Left
        if( xSpeed <= -0.1)
        {
            leftFront = -xSpeed*driveSpeed;
            leftRear = xSpeed*driveSpeed;
            rightFront = xSpeed*driveSpeed;
            rightRear = -xSpeed*driveSpeed;
        }

        //Move Foreward
        if(ySpeed >= 0.1)
        {
            leftFront=  ySpeed*driveSpeed;
            leftRear = ySpeed*driveSpeed;
            rightFront = ySpeed*driveSpeed;
            rightRear = ySpeed*driveSpeed;
        }

        //Move Backward
        if(ySpeed <= -0.1)
        {
            leftFront= ySpeed*driveSpeed;
            leftRear = ySpeed*driveSpeed;
            rightFront = ySpeed*driveSpeed;
            rightRear = ySpeed*driveSpeed;
        }

        //Rotate clockwise
        if(rSpeed <= 0.1)
        {
            leftFront = -rSpeed * driveSpeed;
            leftRear = -rSpeed *driveSpeed;
            rightFront = rSpeed * driveSpeed;
            rightRear = rSpeed*driveSpeed;
        }

        //Rotate Counterclockwise
        if (rSpeed >= -0.1)
        {
            leftFront = -rSpeed * driveSpeed;
            leftRear = -rSpeed *driveSpeed;
            rightFront = rSpeed * driveSpeed;
            rightRear = rSpeed*driveSpeed;
        }

        //Move Diagnally Up Right (maybe)
        if(xSpeed >= 0.1 && ySpeed >= 0.1)
        {
            leftFront = (xSpeed/2) *(ySpeed/2)*driveSpeed;
            leftRear = xSpeed * (ySpeed) * driveSpeed;
            rightFront = xSpeed * (ySpeed) * driveSpeed;
            rightRear = (xSpeed/2) * (ySpeed/2)* driveSpeed;
        }

        //Move Diagnally Up left (maybe)
        if(xSpeed <= -0.1 && ySpeed >= 0.1)
        {
            leftFront = xSpeed *(ySpeed)*driveSpeed;
            leftRear = (xSpeed/2) * (ySpeed/2) * driveSpeed;
            rightFront = (xSpeed/2) * (ySpeed/2) * driveSpeed;
            rightRear = xSpeed * (ySpeed)* driveSpeed;
        }

        //Move Diagnally Down right (maybe)
        if(xSpeed >= 0.1 && ySpeed <= -0.1)
        {
            leftFront = (xSpeed/2) *(ySpeed/2)*driveSpeed;
            leftRear = xSpeed * (ySpeed) * driveSpeed;
            rightFront = xSpeed * (ySpeed) * driveSpeed;
            rightRear = (xSpeed/2) * (ySpeed/2)* driveSpeed;
        }

        // Move Diagonally Down Left (maybe)
        if( xSpeed <= -0.1 && ySpeed <= -0.1)
        {
            leftFront = (xSpeed) * (ySpeed) * driveSpeed;
            leftRear = (xSpeed/2) * (ySpeed/2) * driveSpeed;
            rightFront = (xSpeed/2)* (ySpeed/2) * driveSpeed;
            rightRear = (xSpeed)* (ySpeed) * driveSpeed;
        }

        targetSpeedLeftFront = leftFront;
        targetSpeedLeftRear = leftRear;
        targetSpeedRightFront = rightFront;
        targetSpeedRightRear = rightRear;

      //  if(gamepad1.left_trigger >= 0.1) {
        //}
       // if(gamepad1.right_trigger >= 0.1) {
        //}
        //if(gamepad2.left_trigger) {
        //    currentLiftSpeed = -gamepad2.left_bumper * LIFT_MAX_DOWN_SPEED;
        //}
        //if(gamepad2.right_trigger) {
        //    currentLiftSpeed = gamepad2.right_bumper * LIFT_MAX_UP_SPEED;
        //}
/*
        currentSpeedArm = 0.0;
        if(gamepad1.left_trigger) {
            currentSpeedArm = -gamepad1.left_bumper * ARM_MAX_DOWN_SPEED;
        }
        if(gamepad1.right_trigger) {
            currentSpeedArm = gamepad1.right_bumper * ARM_MAX_UP_SPEED;
        }

        /*
        // limit acceleration by only allowing MAX_ACCEL power change per UPDATE_TIME
        if(runtime.seconds() > UPDATE_TIME)
        {
            runtime.reset();
            
            if( ((currentSpeedLeft >= 0.0) && (targetSpeedLeft >= 0.0)) ||
                    ((currentSpeedLeft <= 0.0) && (targetSpeedLeft <= 0.0)))
            {
                if(Math.abs(currentSpeedLeft) > Math.abs(targetSpeedLeft)) 
                {
                    currentSpeedLeft = targetSpeedLeft;
                }
                else 
                {
                    if (currentSpeedLeft != targetSpeedLeft) {
                        if (Math.abs(targetSpeedLeft - currentSpeedLeft) <= MAX_ACCEL)
                            currentSpeedLeft = targetSpeedLeft;
                        else {
                            if (currentSpeedLeft < targetSpeedLeft)
                                currentSpeedLeft += MAX_ACCEL;
                            else
                                currentSpeedLeft -= MAX_ACCEL;
                        }
                    }
                }

            }
            else 
            {
                currentSpeedLeft = 0.0;
            }

            if( ((currentSpeedRight >= 0.0) && (targetSpeedRight >= 0.0)) ||
                    ((currentSpeedRight <= 0.0) && (targetSpeedRight <= 0.0)))
            {
                if(Math.abs(currentSpeedRight) > Math.abs(targetSpeedRight))
                {
                    currentSpeedRight = targetSpeedRight;
                }
                else
                {
                    if (currentSpeedRight != targetSpeedRight) {
                        if (Math.abs(targetSpeedRight - currentSpeedRight) <= MAX_ACCEL)
                            currentSpeedRight = targetSpeedRight;
                        else {
                            if (currentSpeedRight < targetSpeedRight)
                                currentSpeedRight += MAX_ACCEL;
                            else
                                currentSpeedRight -= MAX_ACCEL;
                        }
                    }
                }

            }
            else
            {
                currentSpeedRight = 0.0;
            }

        }
        */

        // replace acceleration limit because no longer needed
        currentSpeedLeftFront = targetSpeedLeftFront;
        currentSpeedRightFront = targetSpeedRightFront;
        currentSpeedLeftRear = targetSpeedLeftRear;
        currentSpeedRightRear = targetSpeedRightRear;

        robot.LFDriveMotor.setPower(currentSpeedLeftFront);
        robot.RFDriveMotor.setPower(currentSpeedRightFront);
        robot.LBDriveMotor.setPower(currentSpeedLeftRear);
        robot.RBDriveMotor.setPower(currentSpeedRightRear);




 /*       if(gamepad1.a)
        {
            if (!aButtonHeld)
            {
                aButtonHeld = true;
                if (servoOpen) {
                }
                else {
                }
            }
        }
        else {
            aButtonHeld = false;
        }
        */

/*
        if(gamepad1.b)
        {
            if (!bButtonHeld)
            {
                bButtonHeld = true;
                reverseMode = !reverseMode;
            }
        }
        else
        {
            bButtonHeld = false;
        }

*/
        if(gamepad1.y)
        {
            if (!yButtonHeld)
            {
                yButtonHeld = true;

                if(driveSpeed == MAX_SPEED) {
                    driveSpeed = 0.3 * MAX_SPEED;
                    slowMode = true;
                }
                else
                    {
                    driveSpeed = MAX_SPEED;
                    slowMode = false;
                }
            }
        }
        else
        {
            yButtonHeld = false;
        }

/*
        if(gamepad1.x)
        {
            if(!xButtonHeld)
            {
        }
        else
        {
            xButtonHeld = false;
        }
*/
     //   telemetry.addData("reverse", reverseMode);
        telemetry.addData("GP1: LS X Value", gamepad1.left_stick_x);
        telemetry.addData(" GP1: LS Y Value", gamepad1.left_stick_y);
        telemetry.addData("GP 1: RS Value:", gamepad1.right_stick_x);
        telemetry.addData("encoderLeftBack", robot.LBDriveMotor.getCurrentPosition());
        telemetry.addData("encoderRightBack", robot.RBDriveMotor.getCurrentPosition());
        telemetry.addData("encoderLeftFront", robot.LFDriveMotor.getCurrentPosition());
        telemetry.addData("encoderRightFront", robot.RFDriveMotor.getCurrentPosition());
       // telemetry.addData("reverseMode", reverseMode);
        telemetry.addData("slowMode", slowMode);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
