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
 * Created by Devin Moreno on 01/06/2017.
 *
 *   /*This Autonomous program assumes the Robot is positioned in the side clesest to the corner
 vortex and aimed at the first (nearest) of the color randomizers. This method has the
 intention of first recognizing what team it is on, finding the robot to the white line of
 previously mentioned razndomizer, straitening itself out and then pushing the correct
 button as accordance with the team color it recognized earlier. From there it will drive
 forward to the white line of the second (farthest) color randomizer, straighten itself up,
 and once again, press the correct color button before turning the correct way to hit the
 cap ball off of the middle post and park.*/

@Autonomous(name = " Time Buttons", group = "Sensors")
@Disabled
public class AutonomousTimeUse extends LinearOpMode {

    private DcMotor LeftWheel;
    private DcMotor RightWheel;
    ModernRoboticsI2cRangeSensor rangeSensor;
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


    boolean beaconFlagLEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false
    boolean FloorRightLEDState = true;
    boolean FloorLeftLEDState = true;

    boolean bool = true;


    @Override
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

        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon

        //As we did not have sensor values at the time of writing this, These variables are used
        //Throughout the code in the place of values. This is essencailly a valuebank and allows for
        //a smoother read anyway

        waitForStart();


        //Extent button pusher, as to face the flag to get the date of color
            /*This assumes that the color sensor is attatched to the button pusher servo and that
            * when the button pusher is fully extended, the color senseor is facing the flag*/
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        sleep(6150);

        LeftWheel.setPower(-.3);
        RightWheel.setPower(0);
        sleep(750);

        leftFloorCache = FloorLeftReader.read(0x04, 1);
        topCache = beaconFlagReader.read(0x04, 1);
        rightFloorCache = FloorRightReader.read(0x04, 1);
        while ((opModeIsActive()) && 1 == 1) {

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            if ((topCache[0] & 0xFF) <= 4 && (topCache[0] & 0xFF) > 0) {
                LeftWheel.setPower(.2);
                RightWheel.setPower(.2);
                sleep(300);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);

                buttonPusher.setPosition(PUSHER_MAX);
                sleep(1000);
                break;
            } else if ((topCache[0] & 0xFF) >= 5) {
                LeftWheel.setPower(.5);
                RightWheel.setPower(.5);
                sleep(200);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);
            }


        }

        while (opModeIsActive() && bool == true) {
            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);
            if (1 <= (leftFloorCache[0] & 0xFF)) {
                LeftWheel.setPower(0);
                bool = false;
            }
            if (1 <= (rightFloorCache[0] & 0xFF)) {
                RightWheel.setPower(0);
                bool = false;

            }

            LeftWheel.setPower(.3);
            RightWheel.setPower(.3);


            //When the FloorSensor falls within the set parameters (i.e. Sees the white line), stop. If
            //the right side has passed, then reverse it, if it hasnt, make it come forward to
            //align on the white line


        }

        while ((opModeIsActive()) && 1 == 1) {

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            if ((topCache[0] & 0xFF) <= 4 && (topCache[0] & 0xFF) > 0) {
                LeftWheel.setPower(.2);
                RightWheel.setPower(.2);
                sleep(300);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);

                buttonPusher.setPosition(PUSHER_MAX);
                sleep(1000);
                break;
            } else if ((topCache[0] & 0xFF) >= 5) {
                LeftWheel.setPower(.5);
                RightWheel.setPower(.5);
                sleep(200);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);
            }


        }

    }


//Though the code in this method could potentially be just in the main method itself, by making it
    //a method it makes life easier in the future bu allowing for future autonomous programs to
    //call upon the this method and keep most efficiency.

}