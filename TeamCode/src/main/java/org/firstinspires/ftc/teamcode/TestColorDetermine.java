package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "color select", group = "Sensors")

public class TestColorDetermine extends LinearOpMode {
    ColorSensor colorSensor;
    DcMotor LeftWheel;
    DcMotor RightWheel;
    ModernRoboticsI2cRangeSensor rangeSensor;
    Servo buttonPusher;
    ColorSensor FloorSensorRight;
    ColorSensor FLoorSensorLeft;


    @Override
    public void runOpMode() throws InterruptedException{
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        buttonPusher = hardwareMap.servo.get("Button Pusher");
        FloorSensorRight = hardwareMap.colorSensor.get("Right color sensor");
        FLoorSensorLeft = hardwareMap.colorSensor.get("Left color sensor");

        buttonPusher.setDirection(Servo.Direction.REVERSE);


        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN,PUSHER_MAX);


        colorSensor.enableLed(false);


        waitForStart();




        /*This Method contains all of the slight modifications needed to accomplish the goal of
         * this autonomous when we are on the blue team */

        //Mostly variables carried over from topside of program, others are added in for the same
        //reason that the values above were all variables. Good for logic Checking though

        buttonPusher.scaleRange(PUSHER_MIN,PUSHER_MAX);

        int didPress;

        double distance = 0;
        double ACCEPTED_DISTANCE_CLOSE = 11 ;
        double ACCEPTED_DISTANCE_FAR = 14 ;
        double FLOOR_ACCEPTED_VAL_MIN = 1;
        double FLOOR_ACCEPTED_VAL_MAX = 30;
        double ALLOWED_FROM_WALL_MIN = 11;
        double ALLOWED_FROM_WALL_MAX = 16;


        //based on about where we have put the color sensor It cannot see the floor however can see
        //the tape albeit at a low value. Hence the reason I put the threshold origionally at 1
        //If however for some reason it can see the tiling, and it is consistant (i.e. both sensors
        //get the same value for it) then change the minimum to that plus 1.


        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        if(colorSensor.blue() > colorSensor.red()){
            LeftWheel.setPower(.1);
            RightWheel.setPower(.1);
            sleep(200);
            LeftWheel.setPower(0);
            RightWheel.setPower(0);


            buttonPusher.setPosition(PUSHER_MAX);
            didPress = 1;
            sleep(1000);
        }
        else{
            didPress = 0;
        }


        //Essencially This next peice needs to loop until it finds and presses the blue button
        //as 1 = 1 is always a true statement, this will continue until the it reads the break line
        while(1 == 1){

            if(didPress == 1){
                break;
            }

            else if (colorSensor.blue() > colorSensor.red()){
                LeftWheel.setPower(.2);
                RightWheel.setPower(.2);
                sleep(300);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);

                buttonPusher.setPosition(PUSHER_MAX);
                sleep(1000);
                break;
            }

            else if (colorSensor.red() > colorSensor.blue()){
                LeftWheel.setPower(.5);
                RightWheel.setPower(.5);
                sleep(200);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);
            }

            else if (colorSensor.red() == colorSensor.blue()){
                LeftWheel.setPower(.2);
                RightWheel.setPower(.2);
                sleep(300);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);
            }
        }
        // go foward full power for one second before pulling the button pusher back in
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        sleep(1000);

        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        buttonPusher.setPosition(PUSHER_MIN);



    }

}