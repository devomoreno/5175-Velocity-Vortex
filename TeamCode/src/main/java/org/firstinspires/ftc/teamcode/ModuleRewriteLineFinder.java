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

@Autonomous(name = "Module Rewrite Line Finder" , group = "Module")

public class ModuleRewriteLineFinder extends LinearOpMode {


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


        double FLOOR_ACCEPTED_VAL_MIN = 1;
        int target;

        int TARGET_INCREASE = 300;
        boolean canBreak = false;


        leftFloorCache = FloorLeftReader.read(0x04, 1);

        rightFloorCache = FloorRightReader.read(0x04, 1);


        if ((leftFloorCache[0] & 0xFF) > 0 && (rightFloorCache[0] & 0xFF) > 0
                && (leftFloorCache[0] & 0xFF) == (rightFloorCache[0] & 0xFF)) {
            FLOOR_ACCEPTED_VAL_MIN = ((rightFloorCache[0] & 0xFF)) + 1;

            telemetry.addData("New Floor accepted val min: ", FLOOR_ACCEPTED_VAL_MIN);

        }


        canBreak = false;
        int omnicientRight;
        int omnicientLeft;
        boolean didCross = false;


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


//TODO: Make the approach slow and use encoders the whole time, probably a for loop
        for(target = 0; (!canBreak) && opModeIsActive(); target += TARGET_INCREASE) {

            telemetry.addLine("We are in teh For loop");
            telemetry.addData("The target currently is: ", target);
            telemetry.update();

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                LeftWheel.setPower(0);
                RightWheel.setPower(0);
                int leftPosition = LeftWheel.getCurrentPosition();
                int rightPosition = RightWheel.getCurrentPosition();
                LeftWheel.setTargetPosition(leftPosition);
                RightWheel.setTargetPosition(rightPosition);

                telemetry.addLine("Lft a line");
                telemetry.addData("Left Wheel Position is: ", LeftWheel.getCurrentPosition());
                telemetry.update();

                canBreak = true;

            } else {
                LeftWheel.setTargetPosition(target);
                RightWheel.setTargetPosition(target);

            }
            LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (LeftWheel.isBusy()) {
                while ((LeftWheel.isBusy()) && opModeIsActive()) {


                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                        LeftWheel.setPower(0);
                        RightWheel.setPower(0);
                        int leftPosition = LeftWheel.getCurrentPosition();
                        int rightPosition = RightWheel.getCurrentPosition();
                        omnicientLeft  = leftPosition;
                        LeftWheel.setTargetPosition(leftPosition);
                        RightWheel.setTargetPosition(rightPosition);
                        LeftWheel.setPower(.3);
                        RightWheel.setPower(.3);
                        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        telemetry.addLine("Lft a line");
                        telemetry.addData("Left Wheel Position is: ", LeftWheel.getCurrentPosition());
                        telemetry.update();


                        canBreak = true;
                    }
                    if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {  // is this in the right place?
                        RightWheel.setPower(0);
                        omnicientRight = RightWheel.getCurrentPosition();
                        RightWheel.setTargetPosition(omnicientRight);
                        didCross = true;
                    }
                }
            }

        }

        canBreak =false;

        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//TODO: Make the approach slow and use encoders the whole time, probably a for loop
        for(target = 0; (!canBreak) && opModeIsActive(); target += TARGET_INCREASE) {

            telemetry.addLine("We are in teh For loop");
            telemetry.addData("The target currently is: ", target);
            telemetry.update();

            if(didCross){
                RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);

            if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {

                RightWheel.setPower(0);
                int rightPosition = RightWheel.getCurrentPosition();
                RightWheel.setTargetPosition(rightPosition);
                telemetry.addLine("Update: There is a line");
                telemetry.addData("Final target is: ", target);
                telemetry.addData("Wheel Position is(Should match above): ", RightWheel.getCurrentPosition());
                telemetry.update();

                canBreak = true;

            } else {
                RightWheel.setTargetPosition(target);

            }
            RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (RightWheel.isBusy()) {
                while ((RightWheel.isBusy()) && opModeIsActive()) {

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {
                        RightWheel.setPower(0);
                        int rightPosition = RightWheel.getCurrentPosition();
                        RightWheel.setTargetPosition(rightPosition);

                        RightWheel.setPower(.3);
                        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightWheel.setTargetPosition(rightPosition);

                        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        telemetry.addLine("Update: There is a line");
                        telemetry.addData("Final target is: ", target);
                        telemetry.addData("Wheel Position (Should match above): ",rightPosition);
                        telemetry.update();

                        canBreak = true;
                    }
                }
            }
        }

        telemetry.addLine("Job Done");
}
}