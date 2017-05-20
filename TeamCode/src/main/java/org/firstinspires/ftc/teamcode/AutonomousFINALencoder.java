package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "FINAL red")

/**
 * Created by d3499 on 9/24/2016.
 */

public class AutonomousFINALencoder extends LinearOpMode {
    private DcMotor LeftWheel;
   private DcMotor RightWheel;
    private DcMotor LLAMA;
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

        LeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LLAMA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LLAMA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        buttonPusher = hardwareMap.servo.get("Button Pusher");
        buttonPusher.setDirection(Servo.Direction.REVERSE);

        beaconFlagSensor = hardwareMap.i2cDevice.get("color sensor");


        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN,PUSHER_MAX);
        waitForStart();
        int x = LLAMA.getCurrentPosition();
        int encoderShoot = 1800;
        int LeftforwardMove = 1683;
        int RightforwardMove = 1678;
        //FIXME get values for these
        int leftApproach =0;
        int rightApproach = 0;
        int toFirstBeacon = 0;
        int toRamp = 0;

        sleep(10000);
        LeftWheel.setPower(.3);
        RightWheel.setPower(.3);

        LeftWheel.setTargetPosition(LeftforwardMove);
        RightWheel.setTargetPosition(RightforwardMove);

        //move forward
        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        telemetry.addData("Left", LeftWheel.getCurrentPosition());
        telemetry.addData("Right", RightWheel.getCurrentPosition());
        LLAMA.setPower(1);
       //Start off by shooting the preloaded ball
       LLAMA.setTargetPosition(x + encoderShoot);
        LLAMA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);

        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftWheel.setPower (1);
        RightWheel.setPower(1);

        LeftWheel.setTargetPosition(leftApproach);
        RightWheel.setTargetPosition(rightApproach);

        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(3000);
        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if ((topCache[0] & 0xFF) <=4  && (topCache[0] & 0xFF) > 0 &&(topCache[0] & 0xFF) <16) {
            int findLine = 100;
            while (((topCache[0] & 0xFF) <=4  && (topCache[0] & 0xFF) > 0 &&
                    (topCache[0] & 0xFF) <16) && opModeIsActive()) {


                LeftWheel.setTargetPosition(findLine);
                RightWheel.setTargetPosition(findLine);

                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(100);

                topCache = beaconFlagReader.read(0x04, 1);

                if ((topCache[0] & 0xFF) >=7 && (topCache[0] & 0xFF) >= 0){
                    telemetry.addLine("Color Detected, Pressing Button...");
                    telemetry.update();
                    buttonPusher.setPosition(PUSHER_MAX);

                    sleep (300);
                    LeftWheel.setPower(1);
                    RightWheel.setPower(1);


                    LeftWheel.setTargetPosition(toFirstBeacon);
                    RightWheel.setTargetPosition(toFirstBeacon);

                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(100);
                    STOP();
                    break;
                }

                findLine = findLine + 100;
            }

        }

        else  if ((topCache[0] & 0xFF) >=7 && (topCache[0] & 0xFF) >= 0) {
            telemetry.addLine("Color Detected, Pressing Button...");
            telemetry.update();
            buttonPusher.setPosition(PUSHER_MAX);
            sleep (300);
            LeftWheel.setPower(1);
            RightWheel.setPower(1);


            LeftWheel.setTargetPosition(toFirstBeacon);
            RightWheel.setTargetPosition(toFirstBeacon);

            LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(100);

            STOP();
        }

        if ((topCache[0] & 0xFF) <=4  && (topCache[0] & 0xFF) > 0 &&(topCache[0] & 0xFF) <16) {
            int findLine = 100;
            while (((topCache[0] & 0xFF) <=4  && (topCache[0] & 0xFF) > 0 &&
                    (topCache[0] & 0xFF) <16) && opModeIsActive()) {


                LeftWheel.setTargetPosition(findLine);
                RightWheel.setTargetPosition(findLine);

                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(100);

                topCache = beaconFlagReader.read(0x04, 1);

                if ((topCache[0] & 0xFF) >=7 && (topCache[0] & 0xFF) >= 0){
                    telemetry.addLine("Color Detected, Pressing Button...");
                    telemetry.update();
                    buttonPusher.setPosition(PUSHER_MAX);

                    sleep (300);
                    LeftWheel.setPower(1);
                    RightWheel.setPower(1);


                    LeftWheel.setTargetPosition(toRamp);
                    RightWheel.setTargetPosition(toRamp);

                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(100);
                    STOP();
                    break;
                }

                findLine = findLine + 100;
            }

        }

        else  if ((topCache[0] & 0xFF) >=7 && (topCache[0] & 0xFF) >= 0) {
            telemetry.addLine("Color Detected, Pressing Button...");
            telemetry.update();
            buttonPusher.setPosition(PUSHER_MAX);
            sleep (300);
            LeftWheel.setPower(1);
            RightWheel.setPower(1);


            LeftWheel.setTargetPosition(toRamp);
            RightWheel.setTargetPosition(toRamp);

            LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(100);

            STOP();
        }

        }



    public void STOP() throws InterruptedException{

        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    }

