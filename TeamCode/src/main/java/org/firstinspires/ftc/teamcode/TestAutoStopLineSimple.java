package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Devin Moreno on 01/12/2017.
 * Used to determine if the robot has adequate enough time to stop once it sees the white line
 *  */

@Autonomous(name = "Line Test Simple" , group = "Sensors")
@Disabled
public class TestAutoStopLineSimple extends LinearOpMode {

    DcMotor LeftWheel;
    DcMotor RightWheel;
    ModernRoboticsI2cRangeSensor rangeSensor;
    Servo buttonPusher;
    byte[] topCache;
    byte[] rightFloorCache;
    byte[] leftFloorCache;

    I2cDevice beaconFlagSensor;
    I2cDevice FloorRight;
    I2cDevice FloorLeft;

    I2cDeviceSynch beaconFlagReader;
    I2cDeviceSynch FloorRightReader;
    I2cDeviceSynch FloorLeftReader;

        //Tracks the mode of the color sensor; Active = true, Passive = false
    boolean FloorRightLEDState = true;
    boolean FloorLeftLEDState = true;


    @Override
    public void runOpMode() throws InterruptedException {

        beaconFlagSensor = hardwareMap.i2cDevice.get("color sensor");
        FloorLeft = hardwareMap.i2cDevice.get("Left color sensor");
        FloorRight = hardwareMap.i2cDevice.get("Right color sensor");

        beaconFlagReader= new I2cDeviceSynchImpl(beaconFlagSensor, I2cAddr.create8bit(0x3c), false);
        FloorLeftReader = new I2cDeviceSynchImpl(FloorLeft, I2cAddr.create8bit(0x3e), false);
        FloorRightReader = new I2cDeviceSynchImpl(FloorRight, I2cAddr.create8bit(0x3a), false);

        beaconFlagReader.engage();
        FloorLeftReader.engage();
        FloorRightReader.engage();


        if(FloorLeftLEDState){
            FloorRightReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        }
        else{
            FloorLeftReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }


        if(FloorRightLEDState){
            FloorRightReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        }
        else{
            FloorRightReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }


        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");


        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        LeftWheel.setDirection(DcMotor.Direction.REVERSE);

        buttonPusher = hardwareMap.servo.get("Button Pusher");
        buttonPusher.setDirection(Servo.Direction.REVERSE);



        waitForStart();


        double distance = 0;
        double ACCEPTED_DISTANCE_CLOSE = 11;
        double ACCEPTED_DISTANCE_FAR = 14;
        double FLOOR_ACCEPTED_VAL_MIN = 1;
        double FLOOR_ACCEPTED_VAL_MAX = 30;
        double ALLOWED_FROM_WALL_MIN = 11;
        double ALLOWED_FROM_WALL_MAX = 16;
        boolean bool = true;

        leftFloorCache = FloorLeftReader.read(0x04, 1);
        topCache = beaconFlagReader.read(0x04, 1);
        rightFloorCache = FloorRightReader.read(0x04, 1);

        //based on about where we have put the color sensor It cannot see the floor however can see
        //the tape albeit at a low value. Hence the reason I put the threshold origionally at 1
        //If however for some reason it can see the tiling, and it is consistant (i.e. both sensors
        //get the same value for it) then change the minimum to that plus 1.

//Sets the range sensor active for the rest of the opmode



            telemetry.addData("Left ",leftFloorCache[0]&0xFF);
            telemetry.addData("Right ", rightFloorCache[0]&0xFF);

            telemetry.update();

    //move toward the white line
        while(opModeIsActive() && bool == true){
            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);

            LeftWheel.setPower(.3);
    RightWheel.setPower(.3);


        //When the FloorSensor falls within the set parameters (i.e. Sees the white line), stop. If
        //the right side has passed, then reverse it, if it hasnt, make it come forward to
        //align on the white line

       if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0]&0xFF)) {
            LeftWheel.setPower(0);
           bool = false;
        }
        if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0]&0xFF)) {
            RightWheel.setPower(0);
            bool = false;

        }


    }

    }
}