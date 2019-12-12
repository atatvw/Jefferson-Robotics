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
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")

public class PushbotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.\
    static final double UPDATE_TIME = 0.25; // seconds between power updates
    static final double MAX_ACCEL = 0.2 ; // maximum power change per updateTime
    static final double MAX_SPEED = 0.75; // maximum drive speed
    static final double LIFT_MAX_DOWN_SPEED = 0.40; // maximum lift speed
    static final double LIFT_MAX_UP_SPEED = 0.40; // maximum lift speed
    static final double ARM_MAX_DOWN_SPEED = 0.40; // maximum arm speed
    static final double ARM_MAX_UP_SPEED = 0.40; // maximum arm speed
    static final double LEFT_SERVO_CLOSED = 0.25;
    static final double LEFT_SERVO_OPEN = 0.75; // CHANGE
    static final double RIGHT_SERVO_CLOSED = 0.85;
    static final double RIGHT_SERVO_OPEN = 0.35 ; // CHANGE

    double          targetSpeedRight = 0.0;
    double          targetSpeedLeft = 0.0;
    double          currentSpeedLeft = 0.0;
    double          currentSpeedRight = 0.0;
    double          currentSpeedArm = 0.0;
    double          currentLiftSpeed = 0.0;


    boolean         reverseMode = false;
    boolean         bButtonHeld = false;
    boolean         yButtonHeld = false;
    boolean         aButtonHeld = false;
    boolean         servoOpen = false;
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

        robot.leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Possibly add a delay
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
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
        currentSpeedRight = 0.0;
        currentSpeedLeft = 0.0;
       // currentSpeedArm = 0.0;
        currentLiftSpeed = 0.0;

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        if(!reverseMode)
        {
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
        }
        else
        {
            left = gamepad1.right_stick_y;
            right = gamepad1.left_stick_y;
        }

        targetSpeedLeft = left * driveSpeed;
        targetSpeedRight = right * driveSpeed;

        currentLiftSpeed = 0.0;
        if(gamepad1.left_trigger >= 0.1) {
            currentLiftSpeed = -LIFT_MAX_DOWN_SPEED;
        }
        if(gamepad1.right_trigger >= 0.1) {
            currentLiftSpeed = LIFT_MAX_DOWN_SPEED;
        }
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
        currentSpeedLeft = targetSpeedLeft;
        currentSpeedRight = targetSpeedRight;

        robot.leftDriveMotor.setPower(currentSpeedLeft);
        robot.rightDriveMotor.setPower(currentSpeedRight);
  //      robot.armMotor.setPower(currentSpeedArm);
        robot.liftMotor.setPower(currentLiftSpeed);


        if(gamepad1.a)
        {
            if (!aButtonHeld)
            {
                aButtonHeld = true;
                if (servoOpen) {
                    servoOpen = false;
                    robot.leftGrabServo.setPosition(LEFT_SERVO_CLOSED);
                    robot.rightGrabServo.setPosition(RIGHT_SERVO_CLOSED);
                }
                else {
                    servoOpen = true;
                    robot.leftGrabServo.setPosition(LEFT_SERVO_OPEN);
                    robot.rightGrabServo.setPosition(RIGHT_SERVO_OPEN);
                }
            }
        }
        else {
            aButtonHeld = false;
        }
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
                else {
                    driveSpeed = MAX_SPEED;
                    slowMode = false;
                }
            }
        }
        else
        {
            yButtonHeld = false;
        }


        telemetry.addData("reverse", reverseMode);

        telemetry.addData("currentLeft", currentSpeedLeft);
        telemetry.addData("currentRight", currentSpeedRight);
   //     telemetry.addData("currentArm", currentSpeedArm);
        telemetry.addData("currentLift", currentLiftSpeed);
        telemetry.addData("encoderLeft", robot.leftDriveMotor.getCurrentPosition());
        telemetry.addData("encoderRight", robot.rightDriveMotor.getCurrentPosition());
        telemetry.addData("reverseMode", reverseMode);
        telemetry.addData("slowMode", slowMode);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
