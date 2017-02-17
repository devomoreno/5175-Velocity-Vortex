package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Time LLAMA and Charge to Pole")
/**
 * Created by William Lord on 1/28/2017.
 */

public class AutonomousTimeLLAMAandCharge extends LinearOpMode {
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

        LLAMA.setDirection(DcMotorSimple.Direction.REVERSE);
        RightWheel.setDirection(DcMotor.Direction.REVERSE);


        buttonPusher = hardwareMap.servo.get("Button Pusher");
        buttonPusher.setDirection(Servo.Direction.REVERSE);

        beaconFlagSensor = hardwareMap.i2cDevice.get("color sensor");



        waitForStart();





       //Start off by shooting the preloaded ball
        LLAMA.setPower(1);
        sleep(3000);
        LLAMA.setPower(0);
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        sleep(3500);
        // Park on the center vortex. Hopefully.
        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        }
    }

