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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "motor1"
 * Motor channel:  Right drive motor:        "motor2"
 */
public class HardwarePushbotMechanum
{
    /* Public OpMode members. */
    public DcMotor  LFDriveMotor   = null;
    public DcMotor  RFDriveMotor  = null;
    public DcMotor  LBDriveMotor  = null;
    public DcMotor  RBDriveMotor  = null;


/*Public Servos.*/

    /*Public Touch Sensor*/

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public HardwarePushbotMechanum(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LFDriveMotor  = hwMap.get(DcMotor.class, "leftFrontDM");
        RFDriveMotor = hwMap.get(DcMotor.class, "rightFrontDM");
        LBDriveMotor = hwMap.get(DcMotor.class, "leftRearDM");
        RBDriveMotor = hwMap.get(DcMotor.class, "rightRearDM");

        //Define and Intit Servos

        //Define and Init Touch Sensor

        //Set Motor Direction
        LFDriveMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        RFDriveMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        LBDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        RBDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        LFDriveMotor.setPower(0);
        RFDriveMotor.setPower(0);
        LBDriveMotor.setPower(0);
        RBDriveMotor.setPower(0);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LFDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
 }