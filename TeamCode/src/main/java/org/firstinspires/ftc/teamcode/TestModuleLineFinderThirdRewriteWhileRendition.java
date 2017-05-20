package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;


/**
 * Created by Devin Moreno on 02/14/2017.
 * Used to determine if the logic used using a for loop to approach the white line will work
 * the main function to test here is the increment of the variable TARGET_INCREASE and the use
 * of the while loop looking for isBusy to stop if the line is reached while trying to reach
 * their new targets
 *  */

@Autonomous(name = "Third Rewrite LineFinder While(R) " , group = "Module")

public class TestModuleLineFinderThirdRewriteWhileRendition extends LinearOpMode {


    private DcMotor RightWheel;
    private DcMotor LeftWheel;


    byte[] rightFloorCache;
    byte[] leftFloorCache;

    I2cDevice FloorRight;
    I2cDevice FloorLeft;

    I2cDeviceSynch FloorRightReader;
    I2cDeviceSynch FloorLeftReader;


    boolean FloorRightLEDState = true;
    boolean FloorLeftLEDState = true;
    boolean isLine = false;
    boolean foundLeft = false;
    boolean foundRight = false;

    private int leftOnLine;
    private int rightOnLine;

    boolean isOnLine;

    //Tracks the mode of the color sensor; Active = true, Passive = false



    @Override
    public void runOpMode() throws InterruptedException {

        RightWheel = hardwareMap.dcMotor.get("RightWheel");

        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FloorLeft = hardwareMap.i2cDevice.get("Left color sensor");
        FloorRight = hardwareMap.i2cDevice.get("Right color sensor");


        FloorLeftReader = new I2cDeviceSynchImpl(FloorLeft, I2cAddr.create8bit(0x3a), false);
        FloorRightReader = new I2cDeviceSynchImpl(FloorRight, I2cAddr.create8bit(0x3e), false);


        FloorLeftReader.engage();
        FloorRightReader.engage();
        //This initialises the led state of the color sensors

        if (FloorLeftLEDState) {
            FloorLeftReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        } else {
            FloorLeftReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }


        if (FloorRightLEDState) {
            FloorRightReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        } else {
            FloorRightReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }

        leftFloorCache = FloorLeftReader.read(0x04, 1);

        rightFloorCache = FloorRightReader.read(0x04, 1);

        telemetry.addData("Left Sees ", leftFloorCache[0] & 0xFF);
        telemetry.addData("Right Sees ", rightFloorCache[0] & 0xFF);
        telemetry.update();


        waitForStart();
/** Begining of Program*/

        double FLOOR_ACCEPTED_VAL_MIN = 1;
        int target;

        int TARGET_INCREASE = 2000;
        boolean canBreak = false;


        leftFloorCache = FloorLeftReader.read(0x04, 1);

        rightFloorCache = FloorRightReader.read(0x04, 1);
/** Check and fix floor value as according to ambient light and floor*/

        if (((leftFloorCache[0] & 0xFF) > 0) && ((rightFloorCache[0] & 0xFF) > 0)
                && (leftFloorCache[0] & 0xFF) == (rightFloorCache[0] & 0xFF)) {
            FLOOR_ACCEPTED_VAL_MIN = ((rightFloorCache[0] & 0xFF)) + 1;

            telemetry.addData("New Floor accepted val min: ", FLOOR_ACCEPTED_VAL_MIN);

        }

/** Have one of the sensors find the line and then stop*/

        RightWheel.setTargetPosition(100);
        LeftWheel.setTargetPosition(100);

        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        target = 101;
        while ((!canBreak) && opModeIsActive()){

            telemetry.addLine("We are in the beginning loop");
            telemetry.addData("The target currently is: ", target);
            telemetry.update();

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);

            if ((leftFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN ||
                    (rightFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN) {

                telemetry.addLine("One side has found the line");
                RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                isLine = true;
                leftFloorCache = FloorLeftReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                telemetry.addLine("I am figuring out which one...");
                if (((leftFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN)) {
                    foundLeft = true;
                    leftOnLine = LeftWheel.getCurrentPosition();
                }

                if ((rightFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN) {
                    foundRight = true;
                    rightOnLine = RightWheel.getCurrentPosition();
                }

                canBreak = true;
            }

            if (isLine==false) {

                telemetry.addLine("We do not see the line... going forward");
                RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightWheel.setTargetPosition(target);
                LeftWheel.setTargetPosition(target);

                RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while ((RightWheel.isBusy() || LeftWheel.isBusy()) && opModeIsActive()) {
                    RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addLine("We are in the isBusy Loop");

                    int Rcount = RightWheel.getCurrentPosition();
                    int Lcount = LeftWheel.getCurrentPosition();
                    if (Rcount % 30 == 1) {
                        telemetry.addLine("Checking...");
                        telemetry.addData("Target ", RightWheel.getTargetPosition());
                        telemetry.addData("Right Count is: ", Rcount);
                        telemetry.addData("Left Count is: ", Lcount);
                        rightFloorCache = FloorRightReader.read(0x04, 1);
                        if ((rightFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN) {

                            telemetry.addLine("We found something");

                            RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            isLine = true;
                            leftFloorCache = FloorLeftReader.read(0x04, 1);

                            if (((leftFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN)) {
                                foundLeft = true;
                                leftOnLine = LeftWheel.getCurrentPosition();
                            }

                            if ((rightFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN) {
                                foundRight = true;
                                rightOnLine = RightWheel.getCurrentPosition();
                            }
                            canBreak = true;
                        }
                    }

                    if (Lcount % 30 == 0) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        if ((leftFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN ) {
                            RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            isLine = true;
                            rightFloorCache = FloorRightReader.read(0x04, 1);
                            if (((leftFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN)) {
                                foundLeft = true;
                                leftOnLine = LeftWheel.getCurrentPosition();
                            }

                            if ((rightFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN) {
                                foundRight = true;
                                rightOnLine = RightWheel.getCurrentPosition();
                            }
                            canBreak = true;
                        }
                    }

                    target = (target + TARGET_INCREASE);

                }
                //FIXME This is the end
            }
        }
        telemetry.addLine("We are out of the for loop");
/**Whichever one does not see the line, correct*/
           if ((isLine) && (foundLeft || foundRight)){

               for (int targetFinish = 0 ; !isOnLine && opModeIsActive(); targetFinish += 10){
                   if (!foundLeft){
                       LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                       LeftWheel.setTargetPosition(targetFinish);
                       LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                       while (LeftWheel.isBusy()){
                           int Lcount = LeftWheel.getCurrentPosition();

                           if (Lcount % 10 == 0) {
                               leftFloorCache = FloorLeftReader.read(0x04, 1);
                               if ((leftFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN ) {
                                   RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                   LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                   isLine = true;
                                   rightFloorCache = FloorRightReader.read(0x04, 1);
                                   if (((leftFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN)) {
                                       foundLeft = true;
                                       leftOnLine = LeftWheel.getCurrentPosition();
                                   }

                                   if ((rightFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN) {
                                       foundRight = true;
                                       rightOnLine = RightWheel.getCurrentPosition();
                                   }

                               }
                       }
                   }
                   if(foundLeft && foundRight){
                       isOnLine = true;
                   }
               }

               if(!foundRight){
                   RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                   RightWheel.setTargetPosition(targetFinish);
                   RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                   while (RightWheel.isBusy()) {
                       int Rcount = RightWheel.getCurrentPosition();

                       if (Rcount % 10 == 0) {
                           rightFloorCache = FloorRightReader.read(0x04, 1);
                           if ((rightFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN) {

                               RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                               LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                               isLine = true;
                               leftFloorCache = FloorLeftReader.read(0x04, 1);

                               if (((leftFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN)) {
                                   foundLeft = true;
                                   leftOnLine = LeftWheel.getCurrentPosition();
                               }

                               if ((rightFloorCache[0] & 0xFF) > FLOOR_ACCEPTED_VAL_MIN) {
                                   foundRight = true;
                                   rightOnLine = RightWheel.getCurrentPosition();
                               }

                           }
                       }
               }
                   if(foundLeft && foundRight){
                       isOnLine = true;
                   }

                   }




    }

           }
           if (isOnLine) {
               telemetry.addLine("We should be on the line...");
           }
    }
}