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

@Autonomous(name = "Time LLAMA then Park")
@Disabled
/**
 * Created by d3499 on 9/24/2016.
 */

public class AutonomousTimeLLAMAandPark extends LinearOpMode {
    DcMotor LeftWheel;
    DcMotor RightWheel;
    DcMotor LLAMA;
    ModernRoboticsI2cRangeSensor rangeSensor;
    Servo buttonPusher;

    byte[] topCache;

    I2cDevice beaconFlagSensor;

    I2cDeviceSynch beaconFlagReader;

    boolean beaconFlagLEDState = true;
    //Tracks the mode of the color sensor; Active = true, Passive = false



    @Override
    public void runOpMode() throws InterruptedException{
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        LLAMA = hardwareMap.dcMotor.get("LLAMA");
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        LLAMA.setDirection(DcMotorSimple.Direction.REVERSE);


        buttonPusher = hardwareMap.servo.get("Button Pusher");
        buttonPusher.setDirection(Servo.Direction.REVERSE);

        beaconFlagSensor = hardwareMap.i2cDevice.get("color sensor");


        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN, PUSHER_MAX);

        beaconFlagReader= new I2cDeviceSynchImpl(beaconFlagSensor, I2cAddr.create8bit(0x3c), false);


        beaconFlagReader.engage();



        //This initialises the led state of the color sensors

        if(beaconFlagLEDState){
            beaconFlagReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        }
        else{
            beaconFlagReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }


        waitForStart();
        int encoderShoot = 0;
        int forwardMove = 0;
        int turnLeast = 0;
        int turnMost = 0;
        int ontoRamp= 0;

        boolean isBlue = false;


        buttonPusher.setPosition(PUSHER_MAX);
        sleep(500);

        topCache = beaconFlagReader.read(0x04, 1);

        //Test the flag color, if it is blue then set isBlue to true, by default this is false (red)
        switch ((topCache[0]&0xFF)){
            case 2:
            case 3:
            case 4: RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                    LeftWheel.setDirection((DcMotorSimple.Direction.REVERSE));
                    isBlue= true; break;

            default: break;}





       //Start off by shooting the preloaded ball
      LLAMA.setPower(1);
        sleep(6500);

        //move forward
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        sleep(7500);

// may need to use isBusy to make it wait and not do everything at once, will see though

        //turn to align to corner vortex

        }
    }

