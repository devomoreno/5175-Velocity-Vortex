package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "Module Second Rewrite Line Finder" , group = "Module")
@Disabled
public class ModuleSecondRewriteLineFinder extends LinearOpMode {


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

        int TARGET_INCREASE = 1000;
        boolean canBreak = false;


        leftFloorCache = FloorLeftReader.read(0x04, 1);

        rightFloorCache = FloorRightReader.read(0x04, 1);


        if ((leftFloorCache[0] & 0xFF) > 0 && (rightFloorCache[0] & 0xFF) > 0
                && (leftFloorCache[0] & 0xFF) == (rightFloorCache[0] & 0xFF)) {
            FLOOR_ACCEPTED_VAL_MIN = ((rightFloorCache[0] & 0xFF)) + 1;

            telemetry.addData("New Floor accepted val min: ", FLOOR_ACCEPTED_VAL_MIN);

        }



    }
}