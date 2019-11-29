package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.Odometry.Odometer2;

import org.firstinspires.ftc.teamcode.Hardware.Extrusion;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;

@TeleOp(name="Tele-Op NJ", group="Linear Opmode")

public class TeleOpFinal extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor driveFrontLeft;  private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;   private DcMotor driveBackRight;
    double m1, m2, m3, m4;
    double x1, x2, y1, y2, s1, s2, s3;


    private DcMotor liftRight;
    private DcMotor liftLeft;
    private boolean liftReturning;

    private DcMotor intakeLeft;
    private DcMotor intakeRight;


    private DigitalChannel liftLimitSwitch;

    private Servo foundationClampFront;
    private Servo foundationClampBack;

    private Drive DriveTrain;

    private Servo blockGripperPaddle;
    private Servo blockGripperU;

    private Servo flipperServoRight;
    private Servo flipperServoLeft;

    private Servo intakeDropper;

    private Servo blockGrabberFront;
    private Servo blockGrabberBack;

    private double  servoTestingPosition1 = 0; //A value between 0 and 1, this is the position of the servo being tested
    private double  servoTestingPosition2 = 0; //A value between 0 and 1, this is the position of the servo being tested

    private double CLOSED_POSSITION_BLOCK_PADDLE = 0.95;
    private double CLOSED_POSSITION_BLOCK_CLAMP = .56;
    private double CAPSTONE_DOWN = 0;
    private double CAPSTONE_UP = 0;
    private double OPEN_POSSITION_BLOCK_CLAMP = 1;
    private double OPEN_POSSITION_BLOCK_PADDLE = .596;

    private double HOOK_DOWN = .876;
    private double HOOK_UP = .412;
    private double FOUNDATION_CLAMP_1_DOWN = 0.95;
    private double FOUNDATION_CLAMP_2_DOWN = 0.198;
    private double LOWER_INTAKE = 0;
    private double LEFT_FLIPPER_SERVO_OUT = 0.5;
    private double RIGHT_FLIPPER_SERVO_OUT = 0.5;


    private boolean G2_RIGHT_BUMPER_RELEASED;
    private int TIME_G2_RIGHT_BUMPER_RELEASED;
    private int TIME_BETWEEN_BUMPER_AND_CLAMP_LOWER = 50;
    private int TIME = 0;


    private double FOUNDATION_CLAMP_1_UP = 0.248;
    private double FOUNDATION_CLAMP_2_UP = 0.923;

    private double LEFT_FLIPPER_SERVO_MID = 0.8;
    private double RIGHT_FLIPPER_SERVO_MID = .2;

    private double LEFT_FLIPPER_SERVO_IN = 1.0;
    private double RIGHT_FLIPPER_SERVO_IN = 0;



    private void initialize(){
        // Initialize all objects declared above
        //MOTOR NAMING SCHEME FOR HARDWARE MAP:
        //Motors should be named as [motor function] + [motor position]
        //Ie a drive motor in front left position is named "driveFrontLeft"
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight"); //done
        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        liftRight = hardwareMap.dcMotor.get("liftRight");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");


        //SERVOS FROM HARDWARE MAP:
        //Use same naming scheme as motors when available, otherwise, use a logical name
        blockGripperPaddle = hardwareMap.servo.get("blockGripperPaddle");
        blockGripperU = hardwareMap.servo.get("blockGripperU");

        flipperServoLeft = hardwareMap.servo.get("flipperServoLeft");
        flipperServoRight = hardwareMap.servo.get("flipperServoRight");

        //this servo (intakeDropper) is continous rotation
        intakeDropper = hardwareMap.servo.get("intakeDropper");

        foundationClampFront = hardwareMap.servo.get("foundationClampFront");
        foundationClampBack = hardwareMap.servo.get("foundationClampBack");

        blockGrabberFront = hardwareMap.servo.get("blockGrabberFront");
        blockGrabberBack = hardwareMap.servo.get("blockGrabberBack");

        liftLimitSwitch = hardwareMap.digitalChannel.get("liftLimitSwitch");

        //Some Housekeeping========================================================================
        liftLimitSwitch.setMode(DigitalChannel.Mode.INPUT);



        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);


        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setDirection(DcMotor.Direction.REVERSE);

        //Setting Starting Servo Positions ==========================================================

        blockGripperPaddle.setPosition(CLOSED_POSSITION_BLOCK_PADDLE);
        blockGripperU.setPosition(CLOSED_POSSITION_BLOCK_CLAMP);

        flipperServoLeft.setPosition(LEFT_FLIPPER_SERVO_IN);
        flipperServoRight.setPosition(RIGHT_FLIPPER_SERVO_IN);

        telemetry.addData("Status", "Initialized - Welcome, Operators");
        telemetry.update();

    }
    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        initialize();
        waitForStart();
        telemetry.addData("Status", "Running - Good Luck, Operators");
        TIME++;
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            TIME++;
            telemetry.addData("Digital Touch Status is:", liftLimitSwitch.getState());
            telemetry.addData("Time variable is:", TIME);
            telemetry.addData("Boolean Close variable is:", G2_RIGHT_BUMPER_RELEASED);

            telemetry.addData("TIME_G2_RIGHT_BUMPER_RELEASED is:", TIME_G2_RIGHT_BUMPER_RELEASED);
            telemetry.addData("TIME_BETWEEN_BUMPER_AND_CLAMP_LOWER is:", TIME_BETWEEN_BUMPER_AND_CLAMP_LOWER);
            telemetry.addData("TIME variable is:", TIME);

            telemetry.addData("Time Boolean is:", ((TIME_G2_RIGHT_BUMPER_RELEASED  + TIME_BETWEEN_BUMPER_AND_CLAMP_LOWER) < TIME));
            telemetry.addData("True/False statement is:", (!gamepad2.right_bumper && ((TIME_G2_RIGHT_BUMPER_RELEASED  + TIME_BETWEEN_BUMPER_AND_CLAMP_LOWER) < TIME)));
            telemetry.update();
            //DRIVE=================================================================================

            y2 = -gamepad1.left_stick_y;
            y1 = -gamepad1.right_stick_y;
            x1 = gamepad1.right_stick_x;
            x2 = gamepad1.left_stick_x;

            if (gamepad1.right_bumper) {
                s1 = 0.6;
            }else {
                s1 = 1.0;
            }
            if (gamepad1.left_bumper) {
                s2 = 0.6;
            }else {
                s2 = 1.0;
            }


            m1 = (y1 - x1) * s1 * s2;
            m2 = (y1 + x1) * s1 * s2;
            m3 = (-y2 - x2) * s1 * s2;
            m4 = (-y2 + x2) * s1 * s2;

            driveFrontRight.setPower(m1);
            driveBackRight.setPower(m2);
            driveFrontLeft.setPower(m3);
            driveBackLeft.setPower(m4);

//VERTICAL EXTRUSION=============================================================================-
            //UNDERPOWRRED
            if (gamepad2.dpad_up) {
                liftRight.setPower(0.8);
                liftLeft.setPower(0.8);
            }else if (gamepad2.dpad_down && liftLimitSwitch.getState()) {
                liftRight.setPower(-0.45);
                liftLeft.setPower(-0.45);
            }else if(!liftReturning) {
                liftRight.setPower(0);
                liftLeft.setPower(0);
            }
            if(gamepad2.dpad_down || gamepad2.dpad_up){
                liftReturning = false;
            }
            if(gamepad2.a){
                liftReturning = true;
            }
            if(liftReturning && liftLimitSwitch.getState()){
                liftRight.setPower(-0.5);
                liftLeft.setPower(-0.5);
            }
            if(!liftLimitSwitch.getState()){
                liftRight.setPower(0);
                liftLeft.setPower(0);
                liftReturning = false;
            }


//BLOCK GRIPPER====================================================================================
/*
           if(gamepad2.right_bumper){ //Opens servo postiions when trigger pressed
               blockGripperU.setPosition(OPEN_POSSITION_BLOCK_CLAMP);
               blockGripperPaddle.setPosition(OPEN_POSSITION_BLOCK_PADDLE);
           }
           else { //if not pressing trigger, closes one servo, waits TIME_G2_RIGHT_TRIGGER_PRESSED, and the closes other servo gripper
               // if pressed open, block clamp intaking block
               blockGripperPaddle.setPosition(CLOSED_POSSITION_BLOCK_PADDLE);
               blockGripperU.setPosition(CLOSED_POSSITION_BLOCK_CLAMP);
           }
           */



            if(gamepad2.right_bumper){ //Opens servo postiions when trigger pressed
                blockGripperU.setPosition(OPEN_POSSITION_BLOCK_CLAMP);
                blockGripperPaddle.setPosition(OPEN_POSSITION_BLOCK_PADDLE);
                G2_RIGHT_BUMPER_RELEASED = true;
            }

            if(!gamepad2.right_bumper){ //if not pressing trigger, closes one servo, waits TIME_G2_RIGHT_TRIGGER_PRESSED, and the closes other servo gripper
                // if pressed open, block clamp intaking block
                blockGripperPaddle.setPosition(CLOSED_POSSITION_BLOCK_PADDLE);
            }
            if(!gamepad2.right_bumper && G2_RIGHT_BUMPER_RELEASED){
                TIME_G2_RIGHT_BUMPER_RELEASED = TIME;
                G2_RIGHT_BUMPER_RELEASED = false;
            }

            if(!gamepad2.right_bumper && ((TIME_G2_RIGHT_BUMPER_RELEASED  + TIME_BETWEEN_BUMPER_AND_CLAMP_LOWER) < TIME)){
                blockGripperU.setPosition(CLOSED_POSSITION_BLOCK_CLAMP);


            }
            //TAKE OUT BEFORE COMPETITION NJ
            if(gamepad1.start){
                blockGripperU.setPosition(CLOSED_POSSITION_BLOCK_CLAMP);
            }


//BLOCK FLIPPER=====================================================================================

            if(gamepad2.left_trigger > 0.2) {
                flipperServoLeft.setPosition(LEFT_FLIPPER_SERVO_MID);
                flipperServoRight.setPosition(RIGHT_FLIPPER_SERVO_MID);
            }
            else if(gamepad2.left_bumper){
                flipperServoLeft.setPosition(LEFT_FLIPPER_SERVO_OUT);
                flipperServoRight.setPosition(RIGHT_FLIPPER_SERVO_OUT);
            }else{
                flipperServoLeft.setPosition(LEFT_FLIPPER_SERVO_IN);
                flipperServoRight.setPosition(RIGHT_FLIPPER_SERVO_IN);
            }
//INTAKE ===========================================================================================
            //UNDERPOWERED***
            intakeLeft.setPower(-gamepad2.left_stick_y*0.7);
            intakeRight.setPower(-gamepad2.right_stick_y*0.7);


            //BACK WALL SERVOS======================================================================


            //FOUNDATION CLAMP:
            if (gamepad1.left_trigger > 0.2){
                foundationClampFront.setPosition(0.95);
                foundationClampBack.setPosition(0.95);
            }else{
                foundationClampFront.setPosition(0.25);
                foundationClampBack.setPosition(0.25);
            }
            //BLOCK GRABBERS:
            if (gamepad1.right_trigger > 0.2){
                blockGrabberFront.setPosition(0.83);
                blockGrabberBack.setPosition(0.4);
            }else{
                blockGrabberFront.setPosition(0.15);
                blockGrabberBack.setPosition(0.97);
            }

            //INTAKE DEPLOY SERVO
            //**UNDERPOWERED**
            if (gamepad1.x) {
                intakeDropper.setPosition(0);
            }
            else{
                intakeDropper.setPosition(.5);

            }


        }



    }




}











//EXTRA CODE:

          /*
          servoTestingPosition1 = intake.getPosition();
          if(gamepad2.dpad_up){
              servoTestingPosition1 += .001;

          }if(gamepad2.dpad_down){
              servoTestingPosition1 -= .001;
          }



          servoTestingPosition2 = testServo2.getPosition();
          if(gamepad2.y){
              servoTestingPosition2 += .001;

          }if(gamepad2.a){
              servoTestingPosition2 -= .001;
          }


          if (gamepad2.x){
              servoTestingPosition1 += 0.001;
              servoTestingPosition2 -= 0.001;
          }if (gamepad2.b){
              servoTestingPosition1 -= 0.001;
              servoTestingPosition2 += 0.001;
          }
          testServo2.setPosition(servoTestingPosition2);
          intake.setPosition(servoTestingPosition1);

          telemetry.addData("Servo1 Testing Position is:", servoTestingPosition1);
          telemetry.addData("Servo1 Position is read as", intake.getPosition());
          telemetry.addData("Servo2 Testing Position is:", servoTestingPosition2);
          telemetry.addData("Servo2 Position is read as", testServo2.getPosition());

           */









          /*

           if(gamepad2.dpad_up) {

               //outake
               OUTAKE_MANUAL = true;
               liftRight.setPower(-gamepad2.left_stick_y * .75);

               liftLeft.setPower(gamepad2.left_stick_y * .75);




           }

           else if(OUTAKE_MANUAL){
               liftRight.setPower(0);
               liftLeft.setPower(0);
           }


           if(gamepad2.right_trigger > .2){ //Opens servo postiions when trigger pressed
               blockGripperU.setPosition(OPEN_POSSITION_BLOCK_CLAMP);
               blockGripperPaddle.setPosition(OPEN_POSSITION_BLOCK_PADDLE);
           }

           if(gamepad2.right_trigger < .2){ //if not pressing trigger, closes one servo, waits TIME_G2_RIGHT_TRIGGER_PRESSED, and the closes other servo gripper
               // if pressed open, block clamp intaking block
               blockGripperPaddle.setPosition(CLOSED_POSSITION_BLOCK_PADDLE);
               G2_RIGHT_TRIGGER_PRESSED = true;
               TIME_G2_RIGHT_TRIGGER_PRESSED = TIME;
           }

           if(G2_RIGHT_TRIGGER_PRESSED && TIME_G2_RIGHT_TRIGGER_PRESSED  + TIME_BETWEEN_RIGHT_TRIIGER_AND_CLAMP_LOWER < TIME){
               G2_RIGHT_TRIGGER_PRESSED = false;
               blockGripperU.setPosition(CLOSED_POSSITION_BLOCK_CLAMP);
           }




           if(gamepad2.y){
               // capstone
               capstone.setPosition(CAPSTONE_DOWN);
           }

           if(gamepad2.a){
               capstone.setPosition(CAPSTONE_UP);
           }

           if(gamepad1.left_bumper){
               // auto block clamp
               autoBlockGrabber.setPosition(HOOK_DOWN);
           }else{
               autoBlockGrabber.setPosition(HOOK_UP);
           }

           if(gamepad1.right_bumper){
               // if open build clamp
               foundation1.setPosition(FOUNDATION_CLAMP_1_DOWN);
               foundation2.setPosition(FOUNDATION_CLAMP_2_DOWN);
           }

           if(gamepad1.right_trigger > .2){
               // open clamp on biuild plate
               foundation1.setPosition(FOUNDATION_CLAMP_1_UP);
               foundation2.setPosition(FOUNDATION_CLAMP_2_UP);
           }

           if(gamepad1.right_stick_button){
               // drop intake
               intake.setPosition(LOWER_INTAKE);
           }

           if(gamepad2.left_bumper){
               // flip out outake
               G2_LEFT_BUMPER_PRESSED = true;
               if(fc == 0) {
                   flipperServoLeft.setPosition(LEFT_FLIPPER_SERVO_OUT);
                   flipperServoRight.setPosition(RIGHT_FLIPPER_SERVO_OUT);
               }else if (fc == 1){
                   flipperServoLeft.setPosition(LEFT_FLIPPER_SERVO_IN);
                   flipperServoRight.setPosition(RIGHT_FLIPPER_SERVO_IN);
               }
           }
           if(G2_LEFT_BUMPER_PRESSED){
               G2_LEFT_BUMPER_PRESSED = false;
               if(fc == 0){
                   fc = 1;
               }else{
                   fc = 0;
               }

           }

           if(gamepad2.x){
               // reset liftRighttrusion
               liftRight.setPower(.1);//this goes down
               liftLeft.setPower(-.1);
               EXTRUSION_RESETING = true;

               OUTAKE_MANUAL = false;
           }

           if(EXTRUSION_RESETING && !liftLimitSwitch.getState()){
               liftRight.setPower(0);
               liftLeft.setPower(0);
               EXTRUSION_RESETING = false;
           }

       }

           */

