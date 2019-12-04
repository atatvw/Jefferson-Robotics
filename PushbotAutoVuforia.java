//Attempt to code in Vufoia

package org.firstinspires.ftc.teamcode;

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@Autonomous(name= "Pushbot Auto Vuforia", group= "Pushbot")
@Disabled


public class PushbotAutoVuforia extends LinearOpMode {
    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // Neverrest 40
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.460;  // was 0.5
    static final double  TURN_SPEED = 0.1;  // was 0.5
    static final double LIFT_MAX_SPEED = 0.25; // maximum arm speed
    static final double LEFT_SERVO_CLOSED = 0.65;
    static final double LEFT_SERVO_OPEN = 0.0;
    static final double RIGHT_SERVO_CLOSED = 0.27;
    static final double RIGHT_SERVO_OPEN = 1.0;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKY_STONE = "Skystone";

    private static final int SCREEN_WIDTH = 1280;
    private static final int SCREEN_HEIGHT = 720;
    private static final int POINTING_TOLERANCE = 50;

    private static final double LOW_POWER = 0.15;
    private static final double MID_POWER = 0.25;
    private static final int CLICKS_TO_TARGET = 500;

    private static final int COUNTS_PER_ROTATION = 360;

    public enum RobotState {
        TARGET_SKYSTONE,
        MOVE_TO_SKYSTONE,
        DELIVER_SKYSTONE,
        DONE,
        TEST,
        ERROR,
    }

    //Vuforia Access Key
    private static final String VUFORIA_KEY = "ARY7be7/////AAABmUFx6fFG0kg0iK8ZGysA3DhZ+U4cegHVU9U5R7XxqHttgB0m7+HJr+056gqp0Pv4irAkFd5uPhpI6Pjn8R+OL13MePng+UXvAljNqs9wiEjaIt7Ef64Ua4gMKUYPXc7ZaeoGAVWRk1yehTdSfr7ylWzdOPgXJKpF2vjp8LlE4C/UMyYvNRGlIz5t1n3lkCyn7BiozW83u2r/FypaazRISRKYmRUXgrutlV9nnkQSKSZVAFg1QTAkkaMbmIvstJGZukD1oFWktRc0pCQz+FyU8dgoRL8Yg1t2ovcw7M46j1a54oGAXHRPPLCprv85XdKpX1yCNjHO93Kc2QYsB+AO2+tWUQV9pPJIv43DT70FBRBi";

    /*Member Variables*/
    //Stores execution state
    //private RobotState myRobotState = TEST;
    private RobotState myRobotState = RobotState.TARGET_SKYSTONE;

    //Vuforia Localization
    private VuforiaLocalizer myVuforia = null;

    //Tenser Flow Object Detector
    private TFObjectDetector myTfod = null;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
initRobot();
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Wait for start
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            switch (myRobotState) {
                case TARGET_SKYSTONE:
                    targetSkystone();
                    break;
                case MOVE_TO_SKYSTONE:
                    moveToSkystone();
                    break;
                case DELIVER_SKYSTONE:
                    deliverSkystone();
                    break;
                case TEST:
                    encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,1000,1000,0.2,true);
                    encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,-1000,1000,0.2, true);
                    myRobotState = RobotState.DONE;
                    break;
                case ERROR:
                    myRobotState = RobotState.DONE;
                    break;
                case DONE:
                    shutdown();
            }
        }
    }

    //Init Robot//

    //Initialize Vuforia, Tenser Flow, and motors
    //Vuforia is required for camera

    private void initRobot() {
        initVuforia();

        if (myRobotState != RobotState.ERROR) {
            initTfod();
        }
        if (myRobotState != RobotState.ERROR) {
            initMotors();
        }
        //Tell driver that init is complete
        telemetry.addData("status", "Initialized");
    }
    /*Init Vuforia Helper*/
    //Configure Vuforia by creating parameters and passing it
    //Configure use of Rear Camera

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        myVuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (myVuforia == null) {
            myRobotState = RobotState.ERROR;
            telemetry.addData("ERROR", "Vuforia Engine did not init");
        }
    }
        /* Init TFOd Helper*/
        //initialize TF Object Detection
        private void initTfod () {
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                int tfodMoniterViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMoniterViewId", "id", hardwareMap.appContext.getPackageName());
                TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMoniterViewId);
                myTfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, myVuforia);
                myTfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKY_STONE);

                if (myTfod != null) {
                    myTfod.activate();
                } else {
                    telemetry.addData("ERROR", "Tensor Flow Lite did not activate");
                    myRobotState = RobotState.ERROR;
                }
            } else {
                telemetry.addData("ERROR", "This device is not compatable with TFlite/TFOD");
                myRobotState = RobotState.ERROR;
            }
        }
        /*init Motor Helper*/
        //init motors
        private void initMotors () {

            robot.leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

      //Target Skystone
    private void targetSkystone(){
        Recognition skyStone = null;

        //Return without changing value, unless Skystone is identified
        List<Recognition> updatedRecognitions = myTfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("State:", "Target Skystone");
            telemetry.update();
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_SKY_STONE)) {
                    skyStone = recognition;
                    break;
                }
            }
            //We found one
            if (skyStone != null) {
                int skyStoneLeftX = (int) skyStone.getLeft();
                int skyStoneRightX = (int) skyStone.getRight();
                int skyStoneCenterX = (skyStoneLeftX +skyStoneRightX) / 2;
                int error = skyStoneCenterX - SCREEN_WIDTH / 2;
                if (Math.abs(error) < POINTING_TOLERANCE) {
                    myRobotState = RobotState.MOVE_TO_SKYSTONE;
                } else {
                    telemetry.addData("Action:", "Turn", error);
                    telemetry.update();
                    int fixError= error / 8;
                    if (error <= SCREEN_WIDTH/2){
                    encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,fixError,-fixError,0.2, true);
                    }else if(error > SCREEN_WIDTH/2){
                        encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,-fixError,fixError,0.2,true);
                    }
                }
            } else {
                telemetry.addData("Status:", "No Skystone found");
                telemetry.update();
            }
        } else {
            idle();
        }
    }

    //Move to Skystone
    private void moveToSkystone(){
        //Move forward Click_to_target
            telemetry.addData("State:", "Moving to Skystone");
            telemetry.update();
            encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,CLICKS_TO_TARGET-1000, CLICKS_TO_TARGET-1000,0.2,true);
            //turn to pick up Skystone
        encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,6500,-6500,0.2,false);
        //Finish moving forward
        encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,1000,1000,0.2,false);
            myRobotState = RobotState.DELIVER_SKYSTONE;
        }

        //Bring Skystone under bridge
        private void deliverSkystone(){
            //Drive to build zone
            telemetry.addData("State","Delivering Skystone");
            telemetry.update();
            robot.leftGrabServo.setPosition(LEFT_SERVO_CLOSED);
            robot.rightGrabServo.setPosition(RIGHT_SERVO_CLOSED);
            encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,1400,-1400,0.2,false);
            encoderDrivePosition(DRIVE_SPEED,DRIVE_SPEED,CLICKS_TO_TARGET,CLICKS_TO_TARGET,0.2,true);
            myRobotState = RobotState.DONE;
        }
        private void shutdown(){
            telemetry.addData("State: ", "Done");


            if (myTfod != null) {
                myTfod.shutdown();
            }
            telemetry.update();
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
