package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "Smart Buttons", group = "LinearOpMode")
@Disabled
public class AutonomousSmartButtons extends LinearOpMode {
    ColorSensor colorSensor;
    DcMotor LeftWheel;
    DcMotor RightWheel;
    ModernRoboticsI2cRangeSensor rangeSensor;
    Servo buttonPusher;
    ColorSensor FloorSensor;

    @Override
    public void runOpMode() throws InterruptedException{
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        LeftWheel.setDirection(DcMotor.Direction.REVERSE);
        buttonPusher = hardwareMap.servo.get("Button Pusher");
        FloorSensor = hardwareMap.colorSensor.get("color sensor");

        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN,PUSHER_MAX);

        //As we did not have sensor values at the time of writing this, These variables are used
        //Throughout the code in the place of values. This is essencailly a valuebank and allows for
        //a smoother read anyway
        boolean isBlue = false;
        double FlagAcceptedValMIN = ;
        double FlagAcceptedValMAX = ;
        double FLOOR_ACCEPTED_VAL_MIN = ;
        double FLOOR_ACCEPTED_VAL_MAX = ;

        colorSensor.enableLed(true);
        FloorSensor.enableLed(true);

        waitForStart();

        //Extent button pusher, as to face the flag to get the date of color
            /*This assumes that the color sensor is attatched to the button pusher servo and that
            * when the button pusher is fully extended, the color senseor is facing the flag*/
        buttonPusher.setPosition(PUSHER_MAX);
        sleep(200);
        //Test the flag color, if it is blue then set isBlue to true, by default this is false (red)
        if(FlagAcceptedValMIN < colorSensor.blue() && colorSensor.blue()< FlagAcceptedValMAX){
            isBlue = true;
        }
        sleep(200);

        //Call method MethodSelect using isBlue as the Boolean Team
        MethodSelect(isBlue);





    }
//Though the code in this method could potentially be just in the main method itself, by making it
    //a method it makes life easieer in the future bu allowing for future autonomous programs to
    //call upon the this method and keep most efficiency.
    public void MethodSelect (boolean Team) throws InterruptedException{
        //Method used to select which Team Method to route to

        //If Team (isBlue) is true, then route to method BlueTeam
        if( Team ){

            BlueTeam();
            //Call Method BlueTeam
        }
        else{

            RedTeam();

        }
    }
    public void BlueTeam() throws InterruptedException{
        /*This Method contains all of the slight modifications needed to accomplish the goal of
         * this autonomous when we are on the blue team */

        //Mostly variables carried over from topside of program, others are added in for the same
        //reason that the values above were all variables. Good for logic Checking though
        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN,PUSHER_MAX);

        double distance = 0;
        double AcceptedDistance = ;
        double FLOOR_ACCEPTED_VAL_MIN = ;
        double FLOOR_ACCEPTED_VAL_MAX = ;
        double BlueAcceptedValMIN = ;
        double BlueAcceptedValMAX = ;
        double RedExpectedValMIN = ;
        double RedExpectedValMAX = ;
//Sets the range sensor active for the rest of the opmode
        while(opModeIsActive()){
            distance = rangeSensor.getDistance(DistanceUnit.CM);
        }


        //Now that everything is initialized, the actions can begin
        //Pull Button Pusher Back into starting position
        buttonPusher.setPosition(PUSHER_MIN);


        //move toward the white line
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        //When the FloorSensor falls within the set parameters (i.e. Sees the white line), stop
        if(FLOOR_ACCEPTED_VAL_MIN < FloorSensor.alpha()
                && FloorSensor.alpha()< FLOOR_ACCEPTED_VAL_MAX){
            LeftWheel.setPower(0);
            RightWheel.setPower(0);
        }

//This assumes starting off from the white line, this first peice is meant to correct its alignment
        LeftWheel.setPower(-1);
        sleep (750);

        //Just resets the commands
        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        //gets the robot within an accepted distance bu inching it toward the wall. This is done by
        //bakcing up, and sending the left wheel foward frst, then matching the right side to the
        //white line
        while(distance > AcceptedDistance){
            LeftWheel.setPower(-.5);
            RightWheel.setPower(-.5);
            sleep(200);
            LeftWheel.setPower(.5);
            RightWheel.setPower(0);
            sleep(250);
            LeftWheel.setPower(0);
            while(!(FLOOR_ACCEPTED_VAL_MIN < FloorSensor.alpha())){
                RightWheel.setPower(.5);
            }
            RightWheel.setPower(0);

        }

        //When it is within the accepted distance, the robot will stop
        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        //Essencially This next peice needs to loop until it finds and presses the blue button
        //as 1 = 1 is always a true statement, this will continue until the it reads the break line
        while(1 == 1){
            if(BlueAcceptedValMIN < colorSensor.blue() && colorSensor.blue()< BlueAcceptedValMAX){
                buttonPusher.setPosition(PUSHER_MAX);
                break;

            }
            else if (RedExpectedValMIN < colorSensor.red() && colorSensor.red() < RedExpectedValMAX){
                LeftWheel.setPower(.5);
                RightWheel.setPower(.5);
                sleep(200);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);
            }
        }
        // go foward full power for one second before pulling the button pusher back in
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        sleep(1000);

        buttonPusher.setPosition(PUSHER_MIN);

        //once coming to the white line of the second color randomizer, stop and start the above
        //process all over again
        if(FLOOR_ACCEPTED_VAL_MIN < FloorSensor.alpha()
                && FloorSensor.alpha()< FLOOR_ACCEPTED_VAL_MAX){
            LeftWheel.setPower(0);
            RightWheel.setPower(0);
        }
        while(distance > AcceptedDistance){
            LeftWheel.setPower(-.5);
            RightWheel.setPower(-.5);
            sleep(200);
            LeftWheel.setPower(.5);
            RightWheel.setPower(0);
            sleep(250);
            LeftWheel.setPower(0);
            while(!(FLOOR_ACCEPTED_VAL_MIN < FloorSensor.alpha())){
                RightWheel.setPower(.5);
            }
            RightWheel.setPower(0);

        }
        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        //Essencially This next peice needs to loop until it finds and presses the blue button
        //as 1 = 1 is always a true statement, this will continue until the it reads the break line
        while(1 == 1){
            if(BlueAcceptedValMIN < colorSensor.blue() && colorSensor.blue()< BlueAcceptedValMAX){
                buttonPusher.setPosition(PUSHER_MAX);
                break;

            }
            else if (RedExpectedValMIN < colorSensor.red()
                    && colorSensor.red() < RedExpectedValMAX){
                LeftWheel.setPower(.5);
                RightWheel.setPower(.5);
                sleep(200);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);
            }
         }
        LeftWheel.setPower(.5);
        RightWheel.setPower(.5);
        sleep(300);
//turn to face the cap ball and center vortex
        LeftWheel.setPower(-1);
        RightWheel.setPower(1);
        sleep(1250);

        //reach the center vortex maybe hitting off the ball and parking on it
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        sleep(3500);
//Stop
        LeftWheel.setPower(0);
        RightWheel.setPower(0);


    }
    public void RedT0eam() throws InterruptedException{
        /*This Method contains all of the slight modifications needed to accomplish the goal of
         * this autonomous when we are on the red team */

        //Mostly variables carried over from topside program, others are added in for the same
        //reason that the values above were all variables. Good for logic Checking though

        //needed to reverse motors so they could put the robot with the button pusher facing the
        //wall with the buttons every time

        LeftWheel.setDirection(DcMotor.Direction.FORWARD);
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN,PUSHER_MAX);

        double distance = 0;
        double AcceptedDistance = ;
        double FLOOR_ACCEPTED_VAL_MIN = ;
        double FLOOR_ACCEPTED_VAL_MAX = ;
        double RED_ACCEPTED_VAL_MIN = ;
        double RED_ACCEPTED_VAL_MAX = ;
        double BLUE_EXPECTED_VAL_MIN = ;
        double BLUE_EXPECTED_VAL_MAX = ;
//Sets the range sensor active for the rest of the opmode
        while(opModeIsActive()){
            distance = rangeSensor.getDistance(DistanceUnit.CM);
        }

        //Now that everything is initialized, the actions can begin

        //Pull Button Pusher Back into starting position
        buttonPusher.setPosition(PUSHER_MIN);

        //move toward the white line
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        //When the FloorSensor falls within the set parameters (i.e. Sees the white line), stop
        if(FLOOR_ACCEPTED_VAL_MIN < FloorSensor.alpha()
                && FloorSensor.alpha()< FLOOR_ACCEPTED_VAL_MAX){
            LeftWheel.setPower(0);
            RightWheel.setPower(0);
        }

//This assumes starting off from the white line, this first peice is meant to correct its alignment
        LeftWheel.setPower(-1);
        sleep (750);

        //Just resets the commands
        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        //gets the robot within an accepted distance bu inching it toward the wall. This is
        //done by bakcing up, and sending the left wheel foward frst, then matching the right
        //side to the white line
        while(distance > AcceptedDistance){
            LeftWheel.setPower(-.5);
            RightWheel.setPower(-.5);
            sleep(200);
            LeftWheel.setPower(.5);
            RightWheel.setPower(0);
            sleep(250);
            LeftWheel.setPower(0);
            while(!(FLOOR_ACCEPTED_VAL_MIN < FloorSensor.alpha())){
                RightWheel.setPower(.5);
            }
            RightWheel.setPower(0);

        }

        //When it is within the accepted distance, the robot will stop
        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        //Essencially This next peice needs to loop until it finds and presses the red button
        //as 1 = 1 is always a true statement, this will continue until the it reads the break
        // line
        while(1 == 1){
            if(RED_ACCEPTED_VAL_MIN < colorSensor.red()
                    && colorSensor.red()< RED_ACCEPTED_VAL_MAX){
                buttonPusher.setPosition(PUSHER_MAX);
                break;

            }
            else if (BLUE_EXPECTED_VAL_MIN < colorSensor.blue()
                    && colorSensor.blue() < BLUE_EXPECTED_VAL_MAX){
                LeftWheel.setPower(.5);
                RightWheel.setPower(.5);
                sleep(200);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);
            }
        }
        // go foward full power for one second before pulling the button pusher back in
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        sleep(1000);

        buttonPusher.setPosition(PUSHER_MIN);

        //once coming to the white line of the second color randomizer, stop and start the above
        //process all over again
        if(FLOOR_ACCEPTED_VAL_MIN < FloorSensor.alpha() &&
                FloorSensor.alpha()< FLOOR_ACCEPTED_VAL_MAX){
            LeftWheel.setPower(0);
            RightWheel.setPower(0);
        }
        while(distance > AcceptedDistance){
            LeftWheel.setPower(-.5);
            RightWheel.setPower(-.5);
            sleep(200);
            LeftWheel.setPower(.5);
            RightWheel.setPower(0);
            sleep(250);
            LeftWheel.setPower(0);
            while(!(FLOOR_ACCEPTED_VAL_MIN < FloorSensor.alpha())){
                RightWheel.setPower(.5);
            }
            RightWheel.setPower(0);

        }
        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        //Essencially This next peice needs to loop until it finds and presses the red button
        //as 1 = 1 is always a true statement, this will continue until the it reads the break
        // line
        while(1 == 1){
            if(RED_ACCEPTED_VAL_MIN < colorSensor.red()
                    && colorSensor.red()< RED_ACCEPTED_VAL_MAX){
                buttonPusher.setPosition(PUSHER_MAX);
                break;

            }
            else if (BLUE_EXPECTED_VAL_MIN < colorSensor.blue()
                    && colorSensor.blue() < BLUE_EXPECTED_VAL_MAX){
                LeftWheel.setPower(.5);
                RightWheel.setPower(.5);
                sleep(200);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);
            }
        }
        LeftWheel.setPower(.5);
        RightWheel.setPower(.5);
        sleep(300);
//turn to face the cap ball and center vortex
        LeftWheel.setPower(-1);
        RightWheel.setPower(1);
        sleep(1250);

        //reach the center vortex maybe hitting off the ball and parking on it
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        sleep(3500);
//Stop
        LeftWheel.setPower(0);
        RightWheel.setPower(0);



    }

}