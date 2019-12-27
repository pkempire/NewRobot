package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Movement.Drive;

@TeleOp(name="V2.0 Teleop", group="Linear Opmode")

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


    private DigitalChannel leftLiftLimitSwitch;
    private DigitalChannel rightLiftLimitSwitch;

    private Servo foundationClampLeft;
    private Servo foundationClampRight;




    private Servo flipperServoRight;
    private Servo flipperServoLeft;

   // private Servo intakeDropper;

    private Servo blockGrabberFront;
    private Servo blockGrabberBack;





    private double LOWER_INTAKE = 0;



    private boolean G2_RIGHT_BUMPER_RELEASED;
    private int TIME_G2_RIGHT_BUMPER_RELEASED;
    private int TIME_BETWEEN_BUMPER_AND_CLAMP_LOWER = 50;
    private int TIME = 0;
    private int lastPressedFlipper = 0; //0 is in position , 1 is out position
    private int liftAverage = 0;
    private int liftMax = 1300;
    private int liftGoal = 0;
    private int blocklevel = 0; //first level corresponds to second block. 0th level is first block.
    private boolean slidesResetting = false;
    private boolean slidesRunning = false;

    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean liftAtBottom = true;
    private int grabberState = 0;
    private boolean liftManualMode = false;








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
        blockGrabberFront = hardwareMap.servo.get("blockGrabberFront");
        blockGrabberBack = hardwareMap.servo.get("blockGrabberBack");

        flipperServoLeft = hardwareMap.servo.get("flipperServoLeft");
        flipperServoRight = hardwareMap.servo.get("flipperServoRight");

        //this servo (intakeDropper) is continous rotation
        //intakeDropper = hardwareMap.servo.get("intakeDropper");

        foundationClampLeft = hardwareMap.servo.get("foundationClampLeft");
        foundationClampRight = hardwareMap.servo.get("foundationClampRight");

        //blockGrabberFront = hardwareMap.servo.get("blockGrabberFront");
        //blockGrabberBack = hardwareMap.servo.get("blockGrabberBack");

        leftLiftLimitSwitch = hardwareMap.digitalChannel.get("leftLiftLimitSwitch");
        rightLiftLimitSwitch = hardwareMap.digitalChannel.get("rightLiftLimitSwitch");

        //Some Housekeeping========================================================================
        leftLiftLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rightLiftLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        //Lift Housekeeping
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftRight.setDirection(DcMotor.Direction.REVERSE);

        //Setting Starting Servo Positions ==========================================================

        //blockGripperPaddle.setPosition(CLOSED_POSSITION_BLOCK_PADDLE);
        //blockGripperU.setPosition(CLOSED_POSSITION_BLOCK_CLAMP);

        //flipperServoLeft.setPosition(LEFT_FLIPPER_SERVO_IN);
        //flipperServoRight.setPosition(RIGHT_FLIPPER_SERVO_IN);

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
            /*
            telemetry.addData("Digital Touch Status is:", liftLimitSwitch.getState());
            telemetry.addData("Time variable is:", TIME);
            telemetry.addData("Boolean Close variable is:", G2_RIGHT_BUMPER_RELEASED);

            telemetry.addData("TIME_G2_RIGHT_BUMPER_RELEASED is:", TIME_G2_RIGHT_BUMPER_RELEASED);
            telemetry.addData("TIME_BETWEEN_BUMPER_AND_CLAMP_LOWER is:", TIME_BETWEEN_BUMPER_AND_CLAMP_LOWER);
            telemetry.addData("TIME variable is:", TIME);

            telemetry.addData("Time Boolean is:", ((TIME_G2_RIGHT_BUMPER_RELEASED  + TIME_BETWEEN_BUMPER_AND_CLAMP_LOWER) < TIME));
            telemetry.addData("True/False statement is:", (!gamepad2.right_bumper && ((TIME_G2_RIGHT_BUMPER_RELEASED  + TIME_BETWEEN_BUMPER_AND_CLAMP_LOWER) < TIME)));

             */

            telemetry.addData("LiftAverage is: ", liftAverage); //important value
            telemetry.addData("LiftGoal is: ", liftGoal);
            telemetry.addData("zero power behavior is", liftLeft.getZeroPowerBehavior());
            telemetry.addData("LiftAtBottom variable says: ", liftAtBottom);
            telemetry.addData("liftManualMode is", liftManualMode);
            telemetry.update();


            //DRIVE=================================================================================

            y2 = -gamepad1.left_stick_y;
            y1 = -gamepad1.right_stick_y;
            x1 = gamepad1.right_stick_x;
            x2 = gamepad1.left_stick_x;

            if (gamepad1.right_bumper) {
                s1 = 0.4;
            } else {
                s1 = 1.0;
            }
            s2 = 1;


            m1 = (y1 - x1) * s1 * s2;
            m2 = (y1 + x1) * s1 * s2;
            m3 = (-y2 - x2) * s1 * s2;
            m4 = (-y2 + x2) * s1 * s2;

           driveFrontRight.setPower(m1);
           driveBackRight.setPower(m2);
            driveFrontLeft.setPower(m3);
            driveBackLeft.setPower(m4);
           /* if (gamepad1.a){
                driveBackLeft.setPower(.3);
            }
            if (gamepad1.b){
                driveFrontLeft.setPower(.3);
            }
            if (gamepad1.y){
                driveBackRight.setPower(.3);
            }
            if (gamepad1.x){
                driveFrontRight.setPower(.3);
            }
*/
//VERTICAL EXTRUSION=============================================================================-
            //UNDERPOWRRED
            ///*

            liftAverage = -(liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / 2;

            if (!leftLiftLimitSwitch.getState() || !rightLiftLimitSwitch.getState()) {
                liftAtBottom = true;
                liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            } else {
                liftAtBottom = false;
            }
            //switches between manual and auto controls
            if (gamepad2.dpad_left || gamepad2.dpad_right) {
                liftManualMode = true;
            }
            if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.a) {
                liftManualMode = false;
            }

            //MANUAL MODE-----------------------------------------------------------------------
            if (liftManualMode == true) {
                if ((gamepad2.dpad_left) && liftAverage < liftMax) {
                    liftRight.setPower(-0.4); //-0.8
                    liftLeft.setPower(-0.4);
                }
                if ((gamepad2.dpad_left) && liftAverage > liftMax) {
                    liftRight.setPower(0);
                    liftLeft.setPower(0);
                }
                if (gamepad2.dpad_right && liftAtBottom == false) {
                    liftRight.setPower(0.3); //0.6
                    liftLeft.setPower(0.3);
                }
                if (gamepad2.dpad_right && liftAtBottom == true) {
                    liftRight.setPower(0);
                    liftLeft.setPower(0);
                }
                if (!gamepad2.dpad_right && !gamepad2.dpad_left) {
                    liftRight.setPower(0);
                    liftLeft.setPower(0);
                }
            }
            //Makes sure no double-presses when spamming dpad up or down
            //AUTO MODE ------------------------------------------------------------------------
            if (!liftManualMode) {
                if (gamepad2.a) {
                    slidesResetting = true;
                    slidesRunning = false;
                    liftGoal = 0;
                    blocklevel = 0;
                }
                if (slidesResetting == true && liftAverage > 100) {
                    liftRight.setPower(0.5);
                    liftLeft.setPower(0.5);
                }
                if (slidesResetting == true && liftAverage < 100) {
                    liftRight.setPower(0.15);
                    liftLeft.setPower(0.15);
                }
                if (liftAtBottom && slidesResetting == true) {
                    slidesResetting = false;
                    liftLeft.setPower(0);
                    liftRight.setPower(0);
                }


                if (gamepad2.dpad_left || gamepad2.dpad_right ) {
                    slidesResetting = false;
                    slidesRunning = false;
                    liftManualMode = true;
                }
                if ( gamepad2.dpad_down || gamepad2.dpad_up){
                    slidesRunning = true;
                }
                if (gamepad2.dpad_up) {
                    dpadUpPressed = true;
                }
                if (!gamepad2.dpad_up && dpadUpPressed) {

                    if (blocklevel == 0){liftGoal =  185;}  if (blocklevel == 1){liftGoal =  350;}  if (blocklevel == 2){liftGoal =  535;}
                    if (blocklevel == 3){liftGoal =  710;}  if (blocklevel == 4){liftGoal =  875;}  if (blocklevel == 5){liftGoal =  1065;}
                    if (blocklevel == 6){liftGoal =  1225;} // if (blocklevel == 7){liftGoal = liftGoal + }  if (blocklevel == 8){liftGoal = liftGoal + }
                    blocklevel = blocklevel + 1;
                    if (liftGoal > liftMax) {
                        liftGoal = liftMax;
                    }
                    dpadUpPressed = false;
                    slidesRunning = true;
                }
                if (liftGoal - liftAverage > 100 && slidesRunning) {
                    liftRight.setPower(-0.6);
                    liftLeft.setPower(-0.6);
                }
                if (liftGoal - liftAverage > 25 && liftGoal - liftAverage < 100 && slidesRunning) {
                    liftRight.setPower(-0.1);
                    liftLeft.setPower(-0.1);
                }
                if (liftGoal - liftAverage > -25 && liftGoal - liftAverage < 25 && slidesRunning) {
                    liftRight.setPower(-0.02);
                    liftLeft.setPower(-0.02);
                }
                if (liftGoal - liftAverage < -25 && slidesRunning) {
                    liftRight.setPower(0);
                    liftLeft.setPower(0);
                }
            }
//                if (gamepad2.dpad_down) {
//                    dpadDownPressed = true;
//                }
//                if (!gamepad2.dpad_down && dpadDownPressed) {
//                    liftGoal = liftGoal - 200;
//                   dpadDownPressed = false;
//                slidesRunning = true;
                //                  //liftLeft.setTargetPosition(liftGoal);
                //                // liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //          }
//
                //          }





           /* }else if (gamepad2.dpad_down && liftLimitSwitch.getState()) {
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

             */


//BLOCK GRIPPER====================================================================================


                if (gamepad2.b) {
                    grabberState = 0;
                    // Receive position back grabber down and front grabber up
                }
                if (gamepad2.x) {
                    grabberState = 1;
                    // deposit position both grabbers are up
                }
                if (gamepad2.y) {
                    grabberState = 2;
                    // clamp position both grabber are down
                }

                if (grabberState == 0) {
                    blockGrabberFront.setPosition(0.33);
                    blockGrabberBack.setPosition(0.7);
                }
                if (grabberState == 1) {
                    blockGrabberBack.setPosition(0.33);
                    blockGrabberFront.setPosition(0.33);
                }
                if (grabberState == 2) {
                    blockGrabberFront.setPosition(0.7);
                    blockGrabberBack.setPosition(0.7);
                }
  /*
            if(gamepad2.right_bumper){ //Opens servo postiions when trigger pressed
                blockGrabberFront.setPosition(0.33);

            }else{
                blockGrabberFront.setPosition(0.7);
            }
            if(gamepad2.right_bumper){ //Opens servo postions when trigger pressed
                blockGrabberBack.setPosition(0.33);

            }else{
                blockGrabberBack.setPosition(0.7);
            }
*/
//BLOCK FLIPPER=====================================================================================

                if (gamepad2.left_trigger > 0.2) {
                    lastPressedFlipper = 0; //0 is in the chassis, 1 is out of the chassis

                }
                if (gamepad2.left_bumper) {
                    lastPressedFlipper = 1; //0 is in the chassis, 1 is out of the chassis
                }
                if (lastPressedFlipper == 0) {
                    flipperServoLeft.setPosition(0.31);
                    flipperServoRight.setPosition(0.69);
                } else if (lastPressedFlipper == 1) {
                    flipperServoLeft.setPosition(0.89);
                    flipperServoRight.setPosition(0.11);
                }


//INTAKE ===========================================================================================
                //
                intakeLeft.setPower(-gamepad2.left_stick_y * 0.8);
                intakeRight.setPower(gamepad2.right_stick_y * 0.8);


                //BACK WALL SERVOS======================================================================


                //FOUNDATION CLAMP:

                if (gamepad1.left_bumper) {
                    foundationClampLeft.setPosition(0.245);
                    foundationClampRight.setPosition(0.76);
                } else {
                    foundationClampLeft.setPosition(0.745);
                    foundationClampRight.setPosition(0.26);
                }
                //BLOCK GRABBERS:


                //INTAKE DEPLOY SERVO
                //**UNDERPOWERED**


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

