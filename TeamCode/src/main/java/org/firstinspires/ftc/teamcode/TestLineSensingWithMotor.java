package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Devin Moreno on 02/14/2017.
 * Used to determine if the logic used using a for loop to approach the white line will work
 * the main function to test here is the increment of the variable TARGET_INCREASE and the use
 * of the while loop looking for isBusy to stop if the line is reached while trying to reach
 * their new targets
 *  */

@Autonomous(name = "Line Sensing with Motor" , group = "Sensors")

public class TestLineSensingWithMotor extends LinearOpMode {


    private DcMotor RightWheel;


    byte[] topCache;


    I2cDevice beaconFlagSensor;


    I2cDeviceSynch beaconFlagReader;



    boolean beaconFlagLEDState = true;

    //Tracks the mode of the color sensor; Active = true, Passive = false



    @Override
    public void runOpMode() throws InterruptedException{

        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        beaconFlagSensor = hardwareMap.i2cDevice.get("color sensor");
        beaconFlagReader= new I2cDeviceSynchImpl(beaconFlagSensor, I2cAddr.create8bit(0x3a), false);
        beaconFlagReader.engage();

        //This initialises the led state of the color sensors

        if(beaconFlagLEDState){
            beaconFlagReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        }
        else{
            beaconFlagReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }



        waitForStart();

        double FLOOR_ACCEPTED_VAL_MIN = 1;
        int target;

        int TARGET_INCREASE = 1000;
        boolean canBreak = false;


        topCache = beaconFlagReader.read(0x04, 1);


        if ((topCache[0] & 0xFF) > 0) {
            FLOOR_ACCEPTED_VAL_MIN = ((topCache[0] & 0xFF)) + 1;

            telemetry.addData ("New Floor accepted val min: ", FLOOR_ACCEPTED_VAL_MIN);

        }




        canBreak = false;

        topCache = beaconFlagReader.read(0x04, 1);


        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        RightWheel.setPower(.3);

//TODO: Make the approach slow and use encoders the whole time, probably a for loop
        for(target = 0; (!canBreak) && opModeIsActive(); target += TARGET_INCREASE) {

            telemetry.addLine("We are in teh For loop");
            telemetry.addData("The target currently is: ", target);
            telemetry.update();

            topCache = beaconFlagReader.read(0x04, 1);


            if (FLOOR_ACCEPTED_VAL_MIN <= (topCache[0] & 0xFF)) {
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

                    topCache = beaconFlagReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= (topCache[0] & 0xFF)) {
                        int rightPosition = RightWheel.getCurrentPosition();
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
        telemetry.addLine("Update: There is a line");
        telemetry.addData("Final target is: " , target);
        telemetry.update();

        sleep(10000);

    }
}