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

@Autonomous(name = "Smart While Buttons", group = "Sensors")
@Disabled
public class AutonomousWhileUse extends LinearOpMode {

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




    @Override
    public void runOpMode() throws InterruptedException{
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
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

        //As we did not have sensor values at the time of writing this, These variables are used
        //Throughout the code in the place of values. This is essencailly a valuebank and allows for
        //a smoother read anyway

        waitForStart();



        //Extent button pusher, as to face the flag to get the date of color
            /*This assumes that the color sensor is attatched to the button pusher servo and that
            * when the button pusher is fully extended, the color senseor is facing the flag*/

        buttonPusher.setPosition(PUSHER_MAX);
        sleep(500);

        leftFloorCache = FloorLeftReader.read(0x04, 1);
        topCache = beaconFlagReader.read(0x04, 1);
        rightFloorCache = FloorRightReader.read(0x04, 1);


        //Test the flag color, if it is blue then set isBlue to true, by default this is false (red)
        switch ((topCache[0]&0xFF)){
            case 2:
            case 3:
            case 4: BlueTeam(); break;
            case 9:
            case 10:
            case 11: RedTeam(); break;

        }



    }
//Though the code in this method could potentially be just in the main method itself, by making it
    //a method it makes life easier in the future bu allowing for future autonomous programs to
    //call upon the this method and keep most efficiency.

    void BlueTeam() throws InterruptedException{
        /*This Method contains all of the slight modifications needed to accomplish the goal of
         * this autonomous when we are on the blue team */
        while(opModeIsActive()) {

            //Mostly variables carried over from topside of program, others are added in for the same
            //reason that the values above were all variables. Good for logic Checking though
            double PUSHER_MIN = 0;
            double PUSHER_MAX = 1;
            buttonPusher.scaleRange(PUSHER_MIN, PUSHER_MAX);

            int didCross = 2;
            int didPress;
            double FLOOR_ACCEPTED_VAL_MIN = 1;
            double distance = 0.0;
            double ACCEPTED_DISTANCE_CLOSE = 11;
            double ACCEPTED_DISTANCE_FAR = 13;
            double ALLOWED_FROM_WALL_MIN = 11;
            double ALLOWED_FROM_WALL_MAX = 16;


            //based on about where we have put the color sensor It cannot see the floor however can see
            //the tape albeit at a low value. Hence the reason I put the threshold origionally at 1
            //If however for some reason it can see the tiling, and it is consistant (i.e. both sensors
            //get the same value for it) then change the minimum to that plus 1.
            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);

            if ((leftFloorCache[0] & 0xFF) > 0 && (rightFloorCache[0] & 0xFF) > 0
                    && (leftFloorCache[0] & 0xFF) == (rightFloorCache[0] & 0xFF)) {
                FLOOR_ACCEPTED_VAL_MIN = ((rightFloorCache[0] & 0xFF)) + 1;

            }
//Sets the range sensor active for the rest of the opmode


            beaconFlagReader.write8(3, 1); // this tell the top color sensor to go into passive mode


            //Now that everything is initialized, the actions can begin
            //Pull Button Pusher Back into starting position
            buttonPusher.setPosition(PUSHER_MIN);
            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);

            LeftWheel.setPower(1);
            RightWheel.setPower(1);
            sleep(100);
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            while ((opModeIsActive()) && !(FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF))) {
                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);
                //When the FloorSensor falls within the set parameters (i.e. Sees the white line), stop. If
                //the right side has passed, then reverse it, if it hasnt, make it come forward to
                //align on the white line
                if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {
                    didCross = 1;
                } else {
                    didCross = 0;
                }

                if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                    break;
                }
                //move toward the white line
                LeftWheel.setPower(.3);
                RightWheel.setPower(.3);


            }
            LeftWheel.setPower(0);


//If for some reason before the robot corrects itself, it ends up too close and would push the
            //button or hit the beacon, it will correct itself slowly out of the unnacceptable
            //range


            switch (didCross) {
                case 1:
                    RightWheel.setPower(-.2);
                    while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);

                        if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                            RightWheel.setPower(0);
                            break;


                        }


                    }
                case 0:
                    RightWheel.setPower(.2);
                    while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);

                        if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                            RightWheel.setPower(0);
                            break;
                        }
                    }
                default:
                    RightWheel.setPower(-.2);
                    sleep(300);
                    RightWheel.setPower(0);
                    break;

            }


//This assumes starting off from the white line, this first peice is meant to correct its alignment


            //Just resets the commands
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            //gets the robot within an accepted distance bu inching it toward the wall. This is done by
            //bakcing up, and sending the left wheel foward frst, then matching the right side to the
            //white line
            distance = rangeSensor.getDistance(DistanceUnit.CM);
            while ((opModeIsActive()) &&ACCEPTED_DISTANCE_FAR < distance) {

                if (ACCEPTED_DISTANCE_FAR > distance) {
                    break;
                }
                LeftWheel.setPower(-.5);
                RightWheel.setPower(-.5);
                sleep(300);
                RightWheel.setPower(0);
                LeftWheel.setPower(0);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF))) {

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);
                    if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                        break;
                    }
                    LeftWheel.setPower(.2);


                }


                LeftWheel.setPower(0);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF))) {
                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF)) {
                        break;
                    }
                    RightWheel.setPower(.2);

                }
                RightWheel.setPower(0);

                distance = rangeSensor.getDistance(DistanceUnit.CM);

            }

            //When it is within the accepted distance, the robot will stop
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            if ((topCache[0] & 0xFF) <= 4 && (topCache[0] & 0xFF) > 0) {
                LeftWheel.setPower(.1);
                RightWheel.setPower(.1);
                sleep(200);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);


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
            // go foward full power for one second before pulling the button pusher back in
            LeftWheel.setPower(.3);
            RightWheel.setPower(.3);
            sleep(1000);
            LeftWheel.setPower(0);
            RightWheel.setPower(0);
            buttonPusher.setPosition(PUSHER_MIN);

            while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF))) {


                //If on the way to the second beacon something causes it to stray or get too close, then
                //correct itself
                distance = rangeSensor.getDistance(DistanceUnit.CM);
                while ((opModeIsActive()) &&(!(FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) ||
                        (opModeIsActive()) &&(!(FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)))) {
                    distance = rangeSensor.getDistance(DistanceUnit.CM);
                    while ((opModeIsActive()) &&distance > ALLOWED_FROM_WALL_MAX) {
                        RightWheel.setPower(.2);
                        LeftWheel.setPower(.3);
                        distance = rangeSensor.getDistance(DistanceUnit.CM);
                    }
                    distance = rangeSensor.getDistance(DistanceUnit.CM);
                    while ((opModeIsActive()) &&distance < ALLOWED_FROM_WALL_MIN) {
                        LeftWheel.setPower(.2);
                        RightWheel.setPower(.3);
                        distance = rangeSensor.getDistance(DistanceUnit.CM);
                    }
                    distance = rangeSensor.getDistance(DistanceUnit.CM);
                    while ((opModeIsActive()) &&distance <= ALLOWED_FROM_WALL_MAX && distance >= ALLOWED_FROM_WALL_MIN) {
                        distance = rangeSensor.getDistance(DistanceUnit.CM);
                        LeftWheel.setPower(.3);
                        RightWheel.setPower(.3);
                    }
                }

                //once coming to the white line of the second color randomizer, stop and start the above
                //process all over again

                if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {
                    didCross = 1;
                } else {
                    didCross = 0;
                }
                if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                    LeftWheel.setPower(0);
                    break;
                }
            }


            switch (didCross) {
                case 1:
                    RightWheel.setPower(-.2);
                    while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);

                        if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                            RightWheel.setPower(0);
                            break;
                        }

                    }
                case 0:
                    RightWheel.setPower(.2);
                    while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);

                        if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                            RightWheel.setPower(0);
                            break;
                        }
                    }
                default:
                    RightWheel.setPower(-.2);
                    sleep(300);
                    RightWheel.setPower(0);
                    break;

            }


//This assumes starting off from the white line, this first peice is meant to correct its alignment


            //Just resets the commands
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            //gets the robot within an accepted distance bu inching it toward the wall. This is done by
            //bakcing up, and sending the left wheel foward frst, then matching the right side to the
            //white line
            distance = rangeSensor.getDistance(DistanceUnit.CM);
            while ((opModeIsActive()) && ACCEPTED_DISTANCE_FAR < distance) {
                LeftWheel.setPower(-.5);
                RightWheel.setPower(-.5);
                sleep(300);
                RightWheel.setPower(0);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                while ((opModeIsActive()) && !(FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF))) {

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);
                    if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                        break;
                    }
                    LeftWheel.setPower(.2);


                }


                LeftWheel.setPower(0);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF))) {
                    if (FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF)) {
                        break;
                    }
                    RightWheel.setPower(.2);
                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);


                }
                RightWheel.setPower(0);

                distance = rangeSensor.getDistance(DistanceUnit.CM);

            }

            //When it is within the accepted distance, the robot will stop
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            if ((topCache[0] & 0xFF) <= 4 && (topCache[0] & 0xFF) > 0) {
                LeftWheel.setPower(.1);
                RightWheel.setPower(.1);
                sleep(200);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);


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
            while ((opModeIsActive()) &&1 == 1) {

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                if (didPress == 1) {
                    break;
                } else if ((topCache[0] & 0xFF) <= 4) {
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
            // go foward full power for one second before pulling the button pusher back in
            LeftWheel.setPower(.3);
            RightWheel.setPower(.3);
            sleep(1000);
            LeftWheel.setPower(0);
            RightWheel.setPower(0);
            buttonPusher.setPosition(PUSHER_MIN);

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
        // hackish failsafe
        // FIXME: opmode stuck in stop()
        while (!opModeIsActive()){
            LeftWheel.setPower(0);
            RightWheel.setPower(0);
            return;
        }
}
    void RedTeam() throws InterruptedException{
        /*This Method contains all of the slight modifications needed to accomplish the goal of
         * this autonomous when we are on the red team */

        //Mostly variables carried over from topside program, others are added in for the same
        //reason that the values above were all variables. Good for logic Checking though

        //needed to reverse motors so they could put the robot with the button pusher facing the
        //wall with the buttons every time
        while(opModeIsActive()) {
            RightWheel.setDirection(DcMotor.Direction.FORWARD);
            LeftWheel.setDirection(DcMotor.Direction.REVERSE);
            double PUSHER_MIN = 0;
            double PUSHER_MAX = 1;
            buttonPusher.scaleRange(PUSHER_MIN, PUSHER_MAX);

            int didCross = 2;
            int didPress;

            double distance = 0;
            double ACCEPTED_DISTANCE_CLOSE = 11;
            double ACCEPTED_DISTANCE_FAR = 14;
            double FLOOR_ACCEPTED_VAL_MIN = 1;
            double ALLOWED_FROM_WALL_MIN = 11;
            double ALLOWED_FROM_WALL_MAX = 16;

//based on about where we have put the color sensor It cannot see the floor however can see
            //the tape albeit at a low value. Hence the reason I put the threshold origionally at 1
            //If however for some reason it can see the tiling, and it is consistant (i.e. both sensors
            //get the same value for it) then change the minimum to that plus 1.

            if ((opModeIsActive()) &&(leftFloorCache[0] & 0xFF) > 0 && (rightFloorCache[0] & 0xFF) > 0
                    && (leftFloorCache[0] & 0xFF) == (rightFloorCache[0] & 0xFF)) {
                FLOOR_ACCEPTED_VAL_MIN = ((rightFloorCache[0] & 0xFF)) + 1;

            }
//Sets the range sensor active for the rest of the opmode

            beaconFlagReader.write8(3, 1); // this tell the top color sensor to go into passive mode

            //Now that everything is initialized, the actions can begin
            //Pull Button Pusher Back into starting position
            buttonPusher.setPosition(PUSHER_MIN);


            //move toward the white line
            LeftWheel.setPower(1);
            RightWheel.setPower(1);
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF))) {
                if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                    break;
                }

                //move toward the white line
                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);
                LeftWheel.setPower(.3);
                RightWheel.setPower(.3);

                //When the FloorSensor falls within the set parameters (i.e. Sees the white line), stop. If
                //the right side has passed, then reverse it, if it hasnt, make it come forward to
                //align on the white line
                if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {
                    didCross = 1;
                } else {
                    didCross = 0;
                }


            }
            LeftWheel.setPower(0);

            distance = rangeSensor.getDistance(DistanceUnit.CM);
//If for some reason before the robot corrects itself, it ends up too close and would push the
            //button or hit the beacon, it will correct itself slowly out of the unnacceptable
            //range
            while ((opModeIsActive()) &&distance < ACCEPTED_DISTANCE_CLOSE) {
                if (distance > ACCEPTED_DISTANCE_CLOSE) {
                    break;
                }
                LeftWheel.setPower(-.6);
                RightWheel.setPower(-.5);
                sleep(200);

                LeftWheel.setPower(0);

                RightWheel.setPower(.2);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                while (!(FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        RightWheel.setPower(0);
                        break;
                    }
                }


            }

            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            switch (didCross) {
                case 1:
                    RightWheel.setPower(-.2);
                    while (!(FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);

                        if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                            RightWheel.setPower(0);
                            break;
                        }

                    }
                case 0:
                    RightWheel.setPower(.2);
                    while (!(FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);

                        if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                            RightWheel.setPower(0);
                            break;
                        }
                    }
                default:
                    RightWheel.setPower(-.2);
                    sleep(300);
                    RightWheel.setPower(0);
                    break;

            }


//This assumes starting off from the white line, this first peice is meant to correct its alignment


            //Just resets the commands
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            //gets the robot within an accepted distance bu inching it toward the wall. This is done by
            //bakcing up, and sending the left wheel foward frst, then matching the right side to the
            //white line
            distance = rangeSensor.getDistance(DistanceUnit.CM);
            while ((opModeIsActive()) &&ACCEPTED_DISTANCE_FAR < distance) {
                LeftWheel.setPower(-.5);
                RightWheel.setPower(-.5);
                sleep(300);
                RightWheel.setPower(0);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF))) {

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                        break;
                    }
                    LeftWheel.setPower(.2);

                }


                LeftWheel.setPower(0);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF))) {

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF)) {
                        break;
                    }
                    RightWheel.setPower(.2);
                }
                RightWheel.setPower(0);

                distance = rangeSensor.getDistance(DistanceUnit.CM);

            }

            //When it is within the accepted distance, the robot will stop
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            if ((topCache[0] & 0xFF) <= 11 && (topCache[0] & 0xFF) > 8) {
                LeftWheel.setPower(.1);
                RightWheel.setPower(.1);
                sleep(200);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);


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
            while ((opModeIsActive()) &&1 == 1) {

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                if (didPress == 1) {
                    break;
                } else if ((topCache[0] & 0xFF) >= 8) {
                    LeftWheel.setPower(.2);
                    RightWheel.setPower(.2);
                    sleep(300);
                    LeftWheel.setPower(0);
                    RightWheel.setPower(0);

                    buttonPusher.setPosition(PUSHER_MAX);
                    sleep(1000);
                    break;
                } else if ((topCache[0] & 0xFF) <= 7) {
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

            while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF))) {


                //If on the way to the second beacon something causes it to stray or get too close, then
                //correct itself
                distance = rangeSensor.getDistance(DistanceUnit.CM);
                while ((opModeIsActive()) &&(!(FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) ||
                        (!(FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)))) {
                    distance = rangeSensor.getDistance(DistanceUnit.CM);
                    while ((opModeIsActive()) &&distance > ALLOWED_FROM_WALL_MAX) {
                        RightWheel.setPower(.2);
                        LeftWheel.setPower(.3);
                        distance = rangeSensor.getDistance(DistanceUnit.CM);
                    }
                    distance = rangeSensor.getDistance(DistanceUnit.CM);
                    while ((opModeIsActive()) &&distance < ALLOWED_FROM_WALL_MIN) {
                        LeftWheel.setPower(.2);
                        RightWheel.setPower(.3);
                        distance = rangeSensor.getDistance(DistanceUnit.CM);
                    }
                    distance = rangeSensor.getDistance(DistanceUnit.CM);
                    while ((opModeIsActive()) &&distance <= ALLOWED_FROM_WALL_MAX && distance >= ALLOWED_FROM_WALL_MIN) {
                        distance = rangeSensor.getDistance(DistanceUnit.CM);
                        LeftWheel.setPower(.3);
                        RightWheel.setPower(.3);
                    }
                }

                //once coming to the white line of the second color randomizer, stop and start the above
                //process all over again

                if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {
                    didCross = 1;
                } else {
                    didCross = 0;
                }
                if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                    LeftWheel.setPower(0);
                    break;
                }
            }


            switch (didCross) {
                case 1:
                    RightWheel.setPower(-.2);
                    while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);

                        if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                            RightWheel.setPower(0);
                            break;
                        }

                    }
                case 0:
                    RightWheel.setPower(.2);
                    while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);

                        if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                            RightWheel.setPower(0);
                            break;
                        }
                    }
                default:
                    RightWheel.setPower(-.2);
                    sleep(300);
                    RightWheel.setPower(0);
                    break;

            }


//This assumes starting off from the white line, this first peice is meant to correct its alignment


            //Just resets the commands
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            //gets the robot within an accepted distance bu inching it toward the wall. This is done by
            //bakcing up, and sending the left wheel foward frst, then matching the right side to the
            //white line
            distance = rangeSensor.getDistance(DistanceUnit.CM);
            while ((opModeIsActive()) && ACCEPTED_DISTANCE_FAR < distance) {
                LeftWheel.setPower(-.5);
                RightWheel.setPower(-.5);
                sleep(300);
                RightWheel.setPower(0);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF))) {

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);
                    if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                        break;
                    }
                    LeftWheel.setPower(.2);


                }


                LeftWheel.setPower(0);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                while ((opModeIsActive()) &&!(FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF))) {

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF)) {
                        break;
                    }
                    RightWheel.setPower(.2);
                }
                RightWheel.setPower(0);

                distance = rangeSensor.getDistance(DistanceUnit.CM);

            }

            //When it is within the accepted distance, the robot will stop
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            if ((topCache[0] & 0xFF) <= 11 && (topCache[0] & 0xFF) > 8) {
                LeftWheel.setPower(.1);
                RightWheel.setPower(.1);
                sleep(200);
                LeftWheel.setPower(0);
                RightWheel.setPower(0);


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
            while ((opModeIsActive()) &&1 == 1) {

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                if (didPress == 1) {
                    break;
                } else if ((topCache[0] & 0xFF) >= 8) {
                    LeftWheel.setPower(.2);
                    RightWheel.setPower(.2);
                    sleep(300);
                    LeftWheel.setPower(0);
                    RightWheel.setPower(0);

                    buttonPusher.setPosition(PUSHER_MAX);
                    sleep(1000);
                    break;
                } else if ((topCache[0] & 0xFF) <= 7) {
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
        while (!opModeIsActive()){
            LeftWheel.setPower(0);
            RightWheel.setPower(0);
            return;
        }
    }
}