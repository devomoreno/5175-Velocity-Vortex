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

@Autonomous(name = "It's about Time")
@Disabled
/**
 * Created by d3499 on 9/24/2016.
 */

public class AutonomousTimeNoSensors extends LinearOpMode {
    private DcMotor LeftWheel;
   private DcMotor RightWheel;
    private DcMotor LLAMA;
    private Servo buttonPusher;

    byte[] topCache;
    I2cDevice beaconFlagSensor;
    I2cDeviceSynch beaconFlagReader;
    boolean beaconFlagLEDState = false;



    //Tracks the mode of the color sensor; Active = true, Passive = false



    @Override
    public void runOpMode() throws InterruptedException{

        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        LLAMA = hardwareMap.dcMotor.get("LLAMA");
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        beaconFlagSensor = hardwareMap.i2cDevice.get("color sensor");
        LeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buttonPusher = hardwareMap.servo.get("Button Pusher");
        buttonPusher.setDirection(Servo.Direction.REVERSE);

        LeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LLAMA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        beaconFlagReader= new I2cDeviceSynchImpl(beaconFlagSensor, I2cAddr.create8bit(0x3c), false);
        beaconFlagReader.engage();

        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN,PUSHER_MAX);

        if(beaconFlagLEDState){
            beaconFlagReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        }
        else{
            beaconFlagReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }


        waitForStart();
        //Go forward to position to shoot
        long iniForward = 0;
        //Shoot LLAMA
       long shootLLAMA=0 ;
        //Turn towards Wall
        long turnToWall= 0;
        //Approach Wall
        long wallApproach= 0;
        //Correct to prepare for button
        long correctForButton= 0;
        long correctForButton2=0 ;
        //Use sensors to press button
        /** This is sensor*/
        //Go forward
        long toSecondButton= 0;
        //Use Sensors to press button
        /** This is sensor*/
        //turn to center

        //charge
        long chargeTime= 0;

        //Go forward to position to shoot
        LeftWheel.setPower(1);
        RightWheel.setPower(1);

        sleep(iniForward);

        STOP();

        //Shoot LLAMA
        LLAMA.setPower(1);

        sleep(shootLLAMA);
STOP();
        //Turn towards Wall
        LeftWheel.setPower(-1);
        RightWheel.setPower(1);

        sleep(turnToWall);

        STOP();
        //Approach Wall
        LeftWheel.setPower(1);
        RightWheel.setPower(1);

        sleep(wallApproach);

        STOP();
        //Correct to prepare for button

        LeftWheel.setPower(1);
        RightWheel.setPower(-1);

        sleep(correctForButton);

        STOP();

        LeftWheel.setPower(1);
        RightWheel.setPower(1);

        sleep(correctForButton2);

        STOP();
        topCache = beaconFlagReader.read(0x04, 1);
        //Use sensors to press button
        if ((topCache[0] & 0xFF) <=4  && (topCache[0] & 0xFF) > 0 &&(topCache[0] & 0xFF) <16) {
            while (((topCache[0] & 0xFF) <=4  && (topCache[0] & 0xFF) > 0 &&
                    (topCache[0] & 0xFF) <16) && opModeIsActive()) {


                LeftWheel.setPower(1);
                RightWheel.setPower(1);
                sleep(100);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);
                topCache = beaconFlagReader.read(0x04, 1);

                if ((topCache[0] & 0xFF) >=7 && (topCache[0] & 0xFF) >= 0){
                    telemetry.addLine("Color Detected, Pressing Button...");
                    telemetry.update();
                    buttonPusher.setPosition(PUSHER_MAX);

                    LeftWheel.setPower(1);
                    RightWheel.setPower(1);

                    sleep(100);

                    STOP();
                    break;
                }
            }

        }

        else  if ((topCache[0] & 0xFF) >=7 && (topCache[0] & 0xFF) >= 0) {
            telemetry.addLine("Color Detected, Pressing Button...");
            telemetry.update();
            buttonPusher.setPosition(PUSHER_MAX);

            LeftWheel.setPower(1);
            RightWheel.setPower(1);

            sleep(100);

            STOP();
        }

        //Go forward
        LeftWheel.setPower(1);
        RightWheel.setPower(1);

        sleep(toSecondButton);

        STOP();
        //Use Sensors to press button
        topCache = beaconFlagReader.read(0x04, 1);
        //Use sensors to press button
        if ((topCache[0] & 0xFF) <=4  && (topCache[0] & 0xFF) > 0 &&(topCache[0] & 0xFF) <16) {
            while (((topCache[0] & 0xFF) <=4  && (topCache[0] & 0xFF) > 0 &&
                    (topCache[0] & 0xFF) <16) && opModeIsActive()) {


                LeftWheel.setPower(1);
                RightWheel.setPower(1);
                sleep(100);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);
                topCache = beaconFlagReader.read(0x04, 1);

                if ((topCache[0] & 0xFF) >=7 && (topCache[0] & 0xFF) >= 0){
                    telemetry.addLine("Color Detected, Pressing Button...");
                    telemetry.update();
                    buttonPusher.setPosition(PUSHER_MAX);

                    LeftWheel.setPower(1);
                    RightWheel.setPower(1);

                    sleep(100);

                    STOP();
                    break;
                }
            }

        }

        else  if ((topCache[0] & 0xFF) >=7 && (topCache[0] & 0xFF) >= 0) {
            telemetry.addLine("Color Detected, Pressing Button...");
            telemetry.update();
            buttonPusher.setPosition(PUSHER_MAX);

            LeftWheel.setPower(1);
            RightWheel.setPower(1);

            sleep(100);

            STOP();
        }

        //turn to center


        //charge
        }

    public void STOP() throws InterruptedException{

        LeftWheel.setPower(0);
        RightWheel.setPower(0);
        LLAMA.setPower(0);

    }

}
