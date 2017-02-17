package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Devin Moreno on 2/2/2017.
 *
 * This is designed to test the module we have currently deciding on what buttons to press
 */

    @Autonomous(name = "Module Press Button", group = "Sensors")


    public class TestModuleColorPress extends LinearOpMode {

    private DcMotor LeftWheel;
    private DcMotor RightWheel;

    private DcMotor LLAMA;
    private ModernRoboticsI2cRangeSensor rangeSensor;
    private Servo buttonPusher;


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


    boolean beaconFlagLEDState = true;
    boolean FloorRightLEDState = true;
    boolean FloorLeftLEDState = true;
    //Tracks the mode of the color sensor; Active = true, Passive = false



    @Override
    public void runOpMode() throws InterruptedException{
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        LLAMA = hardwareMap.dcMotor.get("LLAMA");

        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LLAMA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LLAMA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        buttonPusher = hardwareMap.servo.get("Button Pusher");
        buttonPusher.setDirection(Servo.Direction.REVERSE);
        beaconFlagSensor = hardwareMap.i2cDevice.get("color sensor");
        FloorLeft = hardwareMap.i2cDevice.get("Left color sensor");
        FloorRight = hardwareMap.i2cDevice.get("Right color sensor");


        beaconFlagReader= new I2cDeviceSynchImpl(beaconFlagSensor, I2cAddr.create8bit(0x3c), false);
        FloorLeftReader = new I2cDeviceSynchImpl(FloorLeft, I2cAddr.create8bit(0x3e), false);
        FloorRightReader = new I2cDeviceSynchImpl(FloorRight, I2cAddr.create8bit(0x3a), false);

        beaconFlagReader.engage();
        FloorLeftReader.engage();
        FloorRightReader.engage();


        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN,PUSHER_MAX);

        //This initialises the led state of the color sensors

        if(beaconFlagLEDState){
            beaconFlagReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        }
        else{
            beaconFlagReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }


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

        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon



        waitForStart();

        int didPress = 2;

        beaconFlagReader.write8(3, 1);

            buttonPusher.setPosition(PUSHER_MIN);

            topCache = beaconFlagReader.read(0x04, 1);

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            if ((topCache[0] & 0xFF) <=4  && (topCache[0] & 0xFF) > 0 &&(topCache[0] & 0xFF) <16) {
                LeftWheel.setPower(.1);
                RightWheel.setPower(.1);
                sleep(100);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);

                telemetry.addLine("Color Detected, Pressing Button...");
                telemetry.update();
                buttonPusher.setPosition(PUSHER_MAX);


                didPress = 1;
                sleep(1000);
            } else {
                didPress = 0;
            }


            //Essencially This next peice needs to loop until it finds and presses the blue button
            //as 1 = 1 is always a true statement, this will continue until the it reads the break line

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);
            while ((opModeIsActive()) && 1 == 1) {

                LeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LLAMA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                if (didPress == 1) {
                    break;
                } else if ((topCache[0] & 0xFF) <= 4 && (topCache[0] & 0xFF) > 0) {
                    LeftWheel.setPower(.2);
                    RightWheel.setPower(.2);
                    sleep(300);
                    LeftWheel.setPower(0);
                    RightWheel.setPower(0);
                    telemetry.addLine("Color Detected, Pressing Button...");
                    telemetry.update();
                    buttonPusher.setPosition(PUSHER_MAX);
                    sleep(1000);
                    break;
                } else if ((topCache[0] & 0xFF) >=7 && (topCache[0] & 0xFF) >= 0) {
                    LeftWheel.setPower(.5);
                    RightWheel.setPower(.5);
                    sleep(200);
                    LeftWheel.setPower(0);
                    RightWheel.setPower(0);
                }


            }
            // go foward full power for one second before pulling the button pusher back in
            LeftWheel.setPower(.3);
            RightWheel.setPower(.3);
            sleep(1000);
            LeftWheel.setPower(0);
            RightWheel.setPower(0);
            buttonPusher.setPosition(PUSHER_MIN);

        }


    }




