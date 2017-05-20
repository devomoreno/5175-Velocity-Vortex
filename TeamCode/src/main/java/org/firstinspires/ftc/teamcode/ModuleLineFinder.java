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

@Autonomous(name = "Module Line Finder" , group = "Module")

public class ModuleLineFinder extends LinearOpMode {


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

    //Tracks the mode of the color sensor; Active = true, Passive = false



    @Override
    public void runOpMode() throws InterruptedException {

        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FloorLeft = hardwareMap.i2cDevice.get("Left color sensor");
        FloorRight = hardwareMap.i2cDevice.get("Right color sensor");


        FloorLeftReader = new I2cDeviceSynchImpl(FloorLeft, I2cAddr.create8bit(0x3a), false);
        FloorRightReader = new I2cDeviceSynchImpl(FloorRight, I2cAddr.create8bit(0x3e), false);


        FloorLeftReader.engage();
        FloorRightReader.engage();
        //This initialises the led state of the color sensors

        if (FloorLeftLEDState) {
            FloorRightReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
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


        double FLOOR_ACCEPTED_VAL_MIN = 1;
        int target;

        int TARGET_INCREASE = 100;
        boolean canBreak = false;


        leftFloorCache = FloorLeftReader.read(0x04, 1);

        rightFloorCache = FloorRightReader.read(0x04, 1);


        if ((leftFloorCache[0] & 0xFF) > 0 && (rightFloorCache[0] & 0xFF) > 0
                && (leftFloorCache[0] & 0xFF) == (rightFloorCache[0] & 0xFF)) {
            FLOOR_ACCEPTED_VAL_MIN = ((rightFloorCache[0] & 0xFF)) + 1;

            telemetry.addData("New Floor accepted val min: ", FLOOR_ACCEPTED_VAL_MIN);

        }


        canBreak = false;
        boolean rightSawLine = false;
        boolean leftSawLine = false;
        leftFloorCache = FloorLeftReader.read(0x04, 1);
        rightFloorCache = FloorRightReader.read(0x04, 1);


        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        RightWheel.setPower(.3);
        LeftWheel.setPower(.3);


        for (target = 1; (!canBreak) && opModeIsActive(); target += TARGET_INCREASE) {
            int rightPosition;
            int leftPosition;

            telemetry.addLine("We are in teh For loop");
            telemetry.addData("The target currently is: ", target);
            telemetry.update();

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) ||
                    (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF))) {

                telemetry.addLine("One saw a line, looking....");

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);
                if (rightSawLine == false) {

                    if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {
                        rightPosition = RightWheel.getCurrentPosition();
                        RightWheel.setTargetPosition(rightPosition);
                        rightSawLine = true;
                        telemetry.addLine("Update: it was right");
                        telemetry.addData("Right Final Position is: ", RightWheel.getCurrentPosition());
                        telemetry.update();
                    } else {
                        RightWheel.setTargetPosition(target);
                    }
                }

                if (leftSawLine == false) {
                    if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                        leftPosition = LeftWheel.getCurrentPosition();
                        LeftWheel.setTargetPosition(leftPosition);
                        leftSawLine = true;
                        telemetry.addLine("Update: It was Left");
                        telemetry.addData("Left Final position is ", LeftWheel.getCurrentPosition());
                        telemetry.update();

                    } else {
                        LeftWheel.setTargetPosition(target);
                    }

                }
            } else {
                if (rightSawLine == false) {
                    RightWheel.setTargetPosition(target);
                }
                if (leftSawLine == false) {
                    LeftWheel.setTargetPosition(target);
                }
                telemetry.addLine("Target is normal");
            }


            RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if (LeftWheel.isBusy()) {
                telemetry.addLine("Left is busy loop");
                while ((LeftWheel.isBusy()) && opModeIsActive()) {

                    rightFloorCache = FloorRightReader.read(0x04, 1);
                    leftFloorCache = FloorRightReader.read(0x04, 1);


                    if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                        if (leftSawLine == false) {
                            leftPosition = LeftWheel.getCurrentPosition();
                            LeftWheel.setTargetPosition(leftPosition);

                            leftSawLine = true;


                            telemetry.addLine("Update: There is a line");
                            telemetry.addData("Final target is: ", target);
                            telemetry.addData("Wheel Position : ", leftPosition);
                            telemetry.update();


                        }
                    }

                    if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {

                        if (rightSawLine == false) {
                            rightPosition = RightWheel.getCurrentPosition();
                            RightWheel.setTargetPosition(rightPosition);

                            rightSawLine = true;

                            RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            telemetry.addLine("Update: There is a line");
                            telemetry.addData("Final target is: ", target);
                            telemetry.addData("Wheel Position (Should match above): ", rightPosition);
                            telemetry.update();


                        }
                    }


                }
            }
            if (RightWheel.isBusy()) {
                telemetry.addLine("Right is busy loop");
                telemetry.update();
                while ((RightWheel.isBusy()) && opModeIsActive()) {

                    rightFloorCache = FloorRightReader.read(0x04, 1);
                    leftFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {

                        if (rightSawLine == false) {
                            rightPosition = RightWheel.getCurrentPosition();
                            RightWheel.setTargetPosition(rightPosition);

                            rightSawLine = true;


                            telemetry.addLine("Update: There is a line");
                            telemetry.addData("Final target is: ", target);
                            telemetry.addData("Wheel Position (Should match above): ", rightPosition);
                            telemetry.update();


                        }

                    }


                    if (leftSawLine && rightSawLine) {
                        canBreak = true;
                    }
                }

                telemetry.addLine("Job Done");
                telemetry.addLine("Update: There is a line");
                telemetry.addData("Final Right is: ", RightWheel.getCurrentPosition());
                telemetry.addData("Final Left is:", LeftWheel.getCurrentPosition());
                telemetry.update();

                sleep(10000);

            }
        }
    }
}