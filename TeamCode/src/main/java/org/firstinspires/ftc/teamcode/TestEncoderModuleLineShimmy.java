package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by d3499 on 1/25/2017.
 *
 * 3rd
 * This module tests the function of going to a white line and then straightening up on it
 */

    @Autonomous(name = "Encoder For Shimmy", group = "Sensors")

    public class TestEncoderModuleLineShimmy extends LinearOpMode {

    DcMotor LeftWheel;
    DcMotor RightWheel;
    ModernRoboticsI2cRangeSensor rangeSensor;
    Servo buttonPusher;

    //Due to the many problems my team is experiencing with the color sensor class, we have decided to
    //try and work with another method of implementing the color sensor. it is the color nuumber
    //system given to us by the modern robotics resources. With a bit of tweaking it now does what
    // we need it to

    byte[] topCache;
    byte[] rightFloorCache;
    byte[] leftFloorCache;

    I2cDevice beaconFlagSensor;
    I2cDevice FloorRight;
    I2cDevice FloorLeft;

    I2cDeviceSynch beaconFlagReader;
    I2cDeviceSynch FloorRightReader;
    I2cDeviceSynch FloorLeftReader;


    boolean beaconFlagLEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false
    boolean FloorRightLEDState = true;
    boolean FloorLeftLEDState = true;


    public void runOpMode() throws InterruptedException {
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        buttonPusher = hardwareMap.servo.get("Button Pusher");
        buttonPusher.setDirection(Servo.Direction.REVERSE);
        beaconFlagSensor = hardwareMap.i2cDevice.get("color sensor");
        FloorLeft = hardwareMap.i2cDevice.get("Left color sensor");
        FloorRight = hardwareMap.i2cDevice.get("Right color sensor");


        beaconFlagReader = new I2cDeviceSynchImpl(beaconFlagSensor, I2cAddr.create8bit(0x3c), false);
        FloorLeftReader = new I2cDeviceSynchImpl(FloorLeft, I2cAddr.create8bit(0x3e), false);
        FloorRightReader = new I2cDeviceSynchImpl(FloorRight, I2cAddr.create8bit(0x3a), false);

        beaconFlagReader.engage();
        FloorLeftReader.engage();
        FloorRightReader.engage();


        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        double distance = rangeSensor.getDistance(DistanceUnit.CM);
        double ACCEPTED_DISTANCE_FAR = 13;

        buttonPusher.scaleRange(PUSHER_MIN, PUSHER_MAX);

        //This initialises the led state of the color sensors

        if (beaconFlagLEDState) {
            beaconFlagReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        } else {
            beaconFlagReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }


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

        //0 = Active - For measuring reflected light. Cancels out ambient light
        //1 = Passive - For measuring ambient light, eg. the FTC Color Beacon

        waitForStart();

        leftFloorCache = FloorLeftReader.read(0x04, 1);
        topCache = beaconFlagReader.read(0x04, 1);
        rightFloorCache = FloorRightReader.read(0x04, 1);


        int FLOOR_ACCEPTED_VAL_MIN = 1;
        boolean canBreak = false;
        int target;
        int TARGET_INCREASE = 10;

        distance = rangeSensor.getDistance(DistanceUnit.CM);
        if (ACCEPTED_DISTANCE_FAR < distance) {
            telemetry.addLine("Distance is a little off, correcting... ");
            telemetry.update();
            while ((opModeIsActive()) && ACCEPTED_DISTANCE_FAR < distance) {

                if (ACCEPTED_DISTANCE_FAR > distance) {

                    telemetry.addLine("Job Done");
                    telemetry.update();
                    break;
                }
                LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                LeftWheel.setPower(-.5);
                RightWheel.setPower(-.5);
                sleep(300);
                RightWheel.setPower(0);
                LeftWheel.setPower(0);


                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);
                LeftWheel.setPower(.3);

                LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                LeftWheel.setPower(.3);


                for (target = 0; (!canBreak) && opModeIsActive(); target += TARGET_INCREASE) {

                    telemetry.addLine("We are in teh For loop");
                    telemetry.addData("The target currently is: ", target);
                    telemetry.update();

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                        int leftPosition = LeftWheel.getCurrentPosition();
                        LeftWheel.setTargetPosition(leftPosition);
                        telemetry.addLine("Update: There is a line");
                        telemetry.addData("Final target is: ", target);
                        telemetry.addData("Wheel Position is(Should match above): ", LeftWheel.getCurrentPosition());
                        telemetry.update();

                        canBreak = true;

                    } else {
                        LeftWheel.setTargetPosition(target);

                    }
                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (LeftWheel.isBusy()) {
                        while ((LeftWheel.isBusy()) && opModeIsActive()) {

                            leftFloorCache = FloorLeftReader.read(0x04, 1);
                            topCache = beaconFlagReader.read(0x04, 1);
                            rightFloorCache = FloorRightReader.read(0x04, 1);

                            if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                                int leftPosition = LeftWheel.getCurrentPosition();
                                LeftWheel.setTargetPosition(leftPosition);
                                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                telemetry.addLine("Update: There is a line");
                                telemetry.addData("Final target is: ", target);
                                telemetry.addData("Wheel Position (Should match above): ", leftPosition);
                                telemetry.update();

                                canBreak = true;
                            }
                        }
                    }
                }
            }
        }
    }
}
