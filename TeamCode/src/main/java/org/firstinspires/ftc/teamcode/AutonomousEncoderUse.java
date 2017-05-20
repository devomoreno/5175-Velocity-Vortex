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

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

@Autonomous(name = "Encoder Buttons", group = "Sensors")
@Disabled
public class AutonomousEncoderUse extends LinearOpMode {

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

        LeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LLAMA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        LeftWheel.setPower(.3);
        RightWheel.setPower(.3);
        sleep(300);
        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        buttonPusher.setPosition(PUSHER_MAX);
        sleep(500);

        leftFloorCache = FloorLeftReader.read(0x04, 1);
        topCache = beaconFlagReader.read(0x04, 1);
        rightFloorCache = FloorRightReader.read(0x04, 1);


        //Test the flag color, if it is blue then set isBlue to true, by default this is false (red)
        switch ((topCache[0]&0xFF)){
            case 2:
            case 3:
            case 4: telemetry.addLine("We are blue");
                    telemetry.update();
                    EncoderBlue();

                    break;

            case 9:
            case 10:
            case 11: LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                     RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                    telemetry.addLine("We are Red:");
                  telemetry.update();
                    EncoderRed(); break;
            default: telemetry.addLine("I didnt see a color, sorry");
                     telemetry.update();
                break;

        }



    }
//Though the code in this method could potentially be just in the main method itself, by making it
    //a method it makes life easier in the future bu allowing for future autonomous programs to
    //call upon the this method and keep most efficiency.

    private void EncoderBlue() throws InterruptedException {
        /*This Method contains all of the slight modifications needed to accomplish the goal of
         * this autonomous when we are on the blue team */
        while (opModeIsActive()) {

            //Mostly variables carried over from topside of program, others are added in for the same
            //reason that the values above were all variables. Good for logic Checking though
            double PUSHER_MIN = 0;
            double PUSHER_MAX = 1;
            buttonPusher.scaleRange(PUSHER_MIN, PUSHER_MAX);

            boolean canBreak = false;

            int didCross = 2;
            int didPress;
            double FLOOR_ACCEPTED_VAL_MIN = 1;
            double distance = 0.0;
            double ACCEPTED_DISTANCE_CLOSE = 11;
            double ACCEPTED_DISTANCE_FAR = 13;
            double ALLOWED_FROM_WALL_MIN = 11;
            double ALLOWED_FROM_WALL_MAX = 16;

            int TARGET_INCREASE = 1;

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

            telemetry.addLine("We're In!");
            telemetry.update();
//Sets the range sensor active for the rest of the opmode


            beaconFlagReader.write8(3, 1); // this tell the top color sensor to go into passive mode


            //Now that everything is initialized, the actions can begin
            //Pull Button Pusher Back into starting position
            buttonPusher.setPosition(PUSHER_MIN);
            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);

            RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            RightWheel.setPower(.3);
            LeftWheel.setPower(.3);
//TODO: Make the approach slow and use encoders the whole time, probably a for loop
            for(int target = 0; (!canBreak) && opModeIsActive(); target += TARGET_INCREASE){
                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {
                    int rightPosition = RightWheel.getCurrentPosition();
                    RightWheel.setTargetPosition(rightPosition);
                }
                else{
                    RightWheel.setTargetPosition(target);
                }

                if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                    int leftPosition = LeftWheel.getCurrentPosition();
                    LeftWheel.setTargetPosition(leftPosition);
                }
                else{
                   LeftWheel.setTargetPosition(target);
                }

                if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF) &&
                    FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                    telemetry.addLine("Breaking While loop Now");
                    telemetry.update();
                    canBreak = true;
                }
                RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
//FIXME
            /*LeftWheel.setPower(.5);
            RightWheel.setPower(.5);

            telemetry.addLine("We should be moving...");
            telemetry.update();


            while ((opModeIsActive()) && canBreak == false) {
                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                telemetry.addLine("First While Loop");
                telemetry.update();

                //When the FloorSensor falls within the set parameters (i.e. Sees the white line), stop. If
                //the right side has passed, then reverse it, if it hasnt, make it come forward to
                //align on the white line
                if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {
                    RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    int rightPosition = RightWheel.getCurrentPosition();
                    RightWheel.setTargetPosition(rightPosition);
                    RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
                if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                    LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    int leftPosition = LeftWheel.getCurrentPosition();
                    LeftWheel.setTargetPosition(leftPosition);
                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }

                if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF) &&
                        FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                    telemetry.addLine("Breaking While loop Now");
                    telemetry.update();
                    canBreak = true;
                }


            }

            LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

*///FIXME


//This assumes starting off from the white line, this first piece is meant to correct its alignment

            //Just resets the commands
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            //gets the robot within an accepted distance by inching it toward the wall. This is done by
            //bakcing up, and sending the left wheel forward first, then matching the right side to the
            //white line
            distance = rangeSensor.getDistance(DistanceUnit.CM);
            if (ACCEPTED_DISTANCE_FAR < distance) {
                telemetry.addLine("Distance is a little off, correcting... ");
                telemetry.update();
                while ((opModeIsActive()) && ACCEPTED_DISTANCE_FAR < distance) {

                    if (ACCEPTED_DISTANCE_FAR > distance) {

                        telemetry.addLine("Job Done");
                        telemetry.update();
                        break;
                    }
                    LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    LeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    LLAMA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    LeftWheel.setPower(-.5);
                    RightWheel.setPower(-.5);
                    sleep(300);
                    RightWheel.setPower(0);
                    LeftWheel.setPower(0);




                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);
                    LeftWheel.setPower(.2);

                    canBreak = false;
                    while ((opModeIsActive()) && canBreak == false) {

                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);
                        if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                            LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            int leftPosition = LeftWheel.getCurrentPosition();
                            LeftWheel.setTargetPosition(leftPosition);
                            LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            canBreak = true;

                        }

                    }

                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    canBreak = false;


                    RightWheel.setPower(.2);

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    while ((opModeIsActive()) && canBreak == false) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);

                        if (FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF)) {
                            RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            int rightPosition = RightWheel.getCurrentPosition();
                            RightWheel.setTargetPosition(rightPosition);
                            canBreak = true;
                        }


                    }
                    RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    distance = rangeSensor.getDistance(DistanceUnit.CM);
                    if (ACCEPTED_DISTANCE_FAR > distance) {
                        break;
                    }

                }

                distance = rangeSensor.getDistance(DistanceUnit.CM);
            }

            telemetry.addLine("Distance looks good, looking for button...");
            telemetry.update();

            LeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LLAMA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //When it is within the accepted distance, the robot will stop
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            if ((topCache[0] & 0xFF) <=4  && (topCache[0] & 0xFF) > 0 &&(topCache[0] & 0xFF) <16) {
                LeftWheel.setPower(.1);
                RightWheel.setPower(.1);
                sleep(200);
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

            LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LLAMA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LLAMA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addLine("Time to go!");
            telemetry.update();


            //If on the way to the second beacon something causes it to stray or get too close, then
            //correct itself
            distance = rangeSensor.getDistance(DistanceUnit.CM);
            while ((opModeIsActive()) && canBreak == false) {

                distance = rangeSensor.getDistance(DistanceUnit.CM);
                while ((opModeIsActive()) && distance > ALLOWED_FROM_WALL_MAX) {

                    LeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    LLAMA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RightWheel.setPower(.2);
                    LeftWheel.setPower(.3);
                    distance = rangeSensor.getDistance(DistanceUnit.CM);

                    if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        int rightPosition = RightWheel.getCurrentPosition();
                        RightWheel.setTargetPosition(rightPosition);
                        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    if ((FLOOR_ACCEPTED_VAL_MIN) <= (leftFloorCache[0] & 0xFF)) {
                        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        int leftPosition = LeftWheel.getCurrentPosition();
                        LeftWheel.setTargetPosition(leftPosition);
                        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }

                    if (LeftWheel.getTargetPosition() > 0 && RightWheel.getTargetPosition() > 0) {
                        canBreak = true;
                        break;
                    }
                }
                LeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LLAMA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                distance = rangeSensor.getDistance(DistanceUnit.CM);
                while ((opModeIsActive()) && distance < ALLOWED_FROM_WALL_MIN) {
                    LeftWheel.setPower(.2);
                    RightWheel.setPower(.3);
                    distance = rangeSensor.getDistance(DistanceUnit.CM);


                    if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        int rightPosition = RightWheel.getCurrentPosition();
                        RightWheel.setTargetPosition(rightPosition);
                        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    if ((FLOOR_ACCEPTED_VAL_MIN) <= (leftFloorCache[0] & 0xFF)) {
                        int leftPosition = LeftWheel.getCurrentPosition();
                        LeftWheel.setTargetPosition(leftPosition);
                        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }

                    if (LeftWheel.getTargetPosition() > 0 && RightWheel.getTargetPosition() > 0) {
                        canBreak = true;
                        break;
                    }
                }
                distance = rangeSensor.getDistance(DistanceUnit.CM);
                while ((opModeIsActive()) && distance <= ALLOWED_FROM_WALL_MAX && distance >= ALLOWED_FROM_WALL_MIN) {
                    distance = rangeSensor.getDistance(DistanceUnit.CM);
                    LeftWheel.setPower(.3);
                    RightWheel.setPower(.3);


                    if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                        int rightPosition = RightWheel.getCurrentPosition();
                        RightWheel.setTargetPosition(rightPosition);
                        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    if ((FLOOR_ACCEPTED_VAL_MIN) <= (leftFloorCache[0] & 0xFF)) {
                        int leftPosition = LeftWheel.getCurrentPosition();
                        LeftWheel.setTargetPosition(leftPosition);
                        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }

                    if (LeftWheel.getTargetPosition() > 0 && RightWheel.getTargetPosition() > 0) {
                        canBreak = true;
                        break;
                    }
                }
            }


            //once coming to the white line of the second color randomizer, stop and start the above
            //process all over again


            distance = rangeSensor.getDistance(DistanceUnit.CM);
            if (ACCEPTED_DISTANCE_FAR < distance) {
                while ((opModeIsActive()) && ACCEPTED_DISTANCE_FAR < distance) {

                    if (ACCEPTED_DISTANCE_FAR > distance) {
                        break;
                    }
                    LeftWheel.setPower(-.5);
                    RightWheel.setPower(-.5);
                    sleep(300);
                    RightWheel.setPower(0);
                    LeftWheel.setPower(0);

                    LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);
                    LeftWheel.setPower(.2);

                    canBreak = false;
                    while ((opModeIsActive()) && canBreak == false) {

                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);
                        if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                            int leftPosition = LeftWheel.getCurrentPosition();
                            LeftWheel.setTargetPosition(leftPosition);
                            LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            canBreak = true;

                        }

                    }

                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    canBreak = false;


                    RightWheel.setPower(.2);

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    while ((opModeIsActive()) && canBreak == false) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);
                        topCache = beaconFlagReader.read(0x04, 1);
                        rightFloorCache = FloorRightReader.read(0x04, 1);

                        if (FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF)) {
                            int rightPosition = RightWheel.getCurrentPosition();
                            RightWheel.setTargetPosition(rightPosition);
                            canBreak = true;
                        }


                    }
                    RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    distance = rangeSensor.getDistance(DistanceUnit.CM);
                    if (ACCEPTED_DISTANCE_FAR > distance) {
                        break;
                    }

                }

                distance = rangeSensor.getDistance(DistanceUnit.CM);
            }

            //When it is within the accepted distance, the robot will stop
            LeftWheel.setPower(0);
            RightWheel.setPower(0);

            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            if ((topCache[0] & 0xFF) <= 11 && (topCache[0] & 0xFF) >= 7) {
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
                } else if ((topCache[0] & 0xFF) >= 7 && (topCache[0] & 0xFF) > 0) {
                    LeftWheel.setPower(.2);
                    RightWheel.setPower(.2);
                    sleep(300);
                    LeftWheel.setPower(0);
                    RightWheel.setPower(0);

                    buttonPusher.setPosition(PUSHER_MAX);
                    sleep(1000);
                    break;
                } else if ((topCache[0] & 0xFF) <= 5 && (topCache[0] & 0xFF) <= 0) {
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

            LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        }
    }
    void EncoderRed() throws InterruptedException{
        /*This Method contains all of the slight modifications needed to accomplish the goal of
         * this autonomous when we are on the red team */

        //Mostly variables carried over from topside program, others are added in for the same
        //reason that the values above were all variables. Good for logic Checking though

        //needed to reverse motors so they could put the robot with the button pusher facing the
        //wall with the buttons every time
        //Mostly variables carried over from topside of program, others are added in for the same
        //reason that the values above were all variables. Good for logic Checking though
        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN, PUSHER_MAX);

        boolean canBreak = false;

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


        LeftWheel.setPower(.5);
        RightWheel.setPower(.5);
        while ((opModeIsActive()) && canBreak == false) {
            leftFloorCache = FloorLeftReader.read(0x04, 1);
            topCache = beaconFlagReader.read(0x04, 1);
            rightFloorCache = FloorRightReader.read(0x04, 1);


            //When the FloorSensor falls within the set parameters (i.e. Sees the white line), stop. If
            //the right side has passed, then reverse it, if it hasnt, make it come forward to
            //align on the white line
            if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {
                int rightPosition = RightWheel.getCurrentPosition();
                RightWheel.setTargetPosition(rightPosition);
                RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                int leftPosition = LeftWheel.getCurrentPosition();
                LeftWheel.setTargetPosition(leftPosition);
                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF) &&
                    FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                canBreak = true;
            }


        }

        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//If for some reason before the robot corrects itself, it ends up too close and would push the
        //button or hit the beacon, it will correct itself slowly out of the unnacceptable
        //range


//This assumes starting off from the white line, this first peice is meant to correct its alignment


        //Just resets the commands
        LeftWheel.setPower(0);
        RightWheel.setPower(0);


        //gets the robot within an accepted distance bu inching it toward the wall. This is done by
        //bakcing up, and sending the left wheel foward frst, then matching the right side to the
        //white line
        distance = rangeSensor.getDistance(DistanceUnit.CM);
        if (ACCEPTED_DISTANCE_FAR < distance) {
            while ((opModeIsActive()) && ACCEPTED_DISTANCE_FAR < distance) {

                if (ACCEPTED_DISTANCE_FAR > distance) {
                    break;
                }
                LeftWheel.setPower(-.5);
                RightWheel.setPower(-.5);
                sleep(300);
                RightWheel.setPower(0);
                LeftWheel.setPower(0);

                LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);
                LeftWheel.setPower(.2);

                canBreak = false;
                while ((opModeIsActive()) && canBreak == false) {

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);
                    if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                        int leftPosition = LeftWheel.getCurrentPosition();
                        LeftWheel.setTargetPosition(leftPosition);
                        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        canBreak = true;

                    }

                }

                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                canBreak = false;


                RightWheel.setPower(.2);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                while ((opModeIsActive()) && canBreak == false) {
                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF)) {
                        int rightPosition = RightWheel.getCurrentPosition();
                        RightWheel.setTargetPosition(rightPosition);
                        canBreak = true;
                    }


                }
                RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                distance = rangeSensor.getDistance(DistanceUnit.CM);
                if (ACCEPTED_DISTANCE_FAR > distance) {
                    break;
                }

            }

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
            } else if ((topCache[0] & 0xFF) >= 7 && (topCache[0] & 0xFF) <= 11) {
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

        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //If on the way to the second beacon something causes it to stray or get too close, then
        //correct itself
        distance = rangeSensor.getDistance(DistanceUnit.CM);
        while ((opModeIsActive()) && canBreak == false) {

            distance = rangeSensor.getDistance(DistanceUnit.CM);
            while ((opModeIsActive()) && distance > ALLOWED_FROM_WALL_MAX) {
                RightWheel.setPower(.2);
                LeftWheel.setPower(.3);
                distance = rangeSensor.getDistance(DistanceUnit.CM);

                if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                    int rightPosition = RightWheel.getCurrentPosition();
                    RightWheel.setTargetPosition(rightPosition);
                    RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if ((FLOOR_ACCEPTED_VAL_MIN) <= (leftFloorCache[0] & 0xFF)) {
                    int leftPosition = LeftWheel.getCurrentPosition();
                    LeftWheel.setTargetPosition(leftPosition);
                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (LeftWheel.getTargetPosition() > 0 && RightWheel.getTargetPosition() > 0) {
                    canBreak = true;
                    break;
                }
            }
            distance = rangeSensor.getDistance(DistanceUnit.CM);
            while ((opModeIsActive()) && distance < ALLOWED_FROM_WALL_MIN) {
                LeftWheel.setPower(.2);
                RightWheel.setPower(.3);
                distance = rangeSensor.getDistance(DistanceUnit.CM);


                if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                    int rightPosition = RightWheel.getCurrentPosition();
                    RightWheel.setTargetPosition(rightPosition);
                    RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if ((FLOOR_ACCEPTED_VAL_MIN) <= (leftFloorCache[0] & 0xFF)) {
                    int leftPosition = LeftWheel.getCurrentPosition();
                    LeftWheel.setTargetPosition(leftPosition);
                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (LeftWheel.getTargetPosition() > 0 && RightWheel.getTargetPosition() > 0) {
                    canBreak = true;
                    break;
                }
            }
            distance = rangeSensor.getDistance(DistanceUnit.CM);
            while ((opModeIsActive()) && distance <= ALLOWED_FROM_WALL_MAX && distance >= ALLOWED_FROM_WALL_MIN) {
                distance = rangeSensor.getDistance(DistanceUnit.CM);
                LeftWheel.setPower(.3);
                RightWheel.setPower(.3);


                if ((FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF))) {
                    int rightPosition = RightWheel.getCurrentPosition();
                    RightWheel.setTargetPosition(rightPosition);
                    RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if ((FLOOR_ACCEPTED_VAL_MIN) <= (leftFloorCache[0] & 0xFF)) {
                    int leftPosition = LeftWheel.getCurrentPosition();
                    LeftWheel.setTargetPosition(leftPosition);
                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (LeftWheel.getTargetPosition() > 0 && RightWheel.getTargetPosition() > 0) {
                    canBreak = true;
                    break;
                }
            }
        }


        //once coming to the white line of the second color randomizer, stop and start the above
        //process all over again


        distance = rangeSensor.getDistance(DistanceUnit.CM);
        if (ACCEPTED_DISTANCE_FAR < distance) {
            while ((opModeIsActive()) && ACCEPTED_DISTANCE_FAR < distance) {

                if (ACCEPTED_DISTANCE_FAR > distance) {
                    break;
                }
                LeftWheel.setPower(-.5);
                RightWheel.setPower(-.5);
                sleep(300);
                RightWheel.setPower(0);
                LeftWheel.setPower(0);

                LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);
                LeftWheel.setPower(.2);

                canBreak = false;
                while ((opModeIsActive()) && canBreak == false) {

                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);
                    if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                        int leftPosition = LeftWheel.getCurrentPosition();
                        LeftWheel.setTargetPosition(leftPosition);
                        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        canBreak = true;

                    }

                }

                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                canBreak = false;


                RightWheel.setPower(.2);

                leftFloorCache = FloorLeftReader.read(0x04, 1);
                topCache = beaconFlagReader.read(0x04, 1);
                rightFloorCache = FloorRightReader.read(0x04, 1);

                while ((opModeIsActive()) && canBreak == false) {
                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= ((rightFloorCache[0] & 0xFF) & 0xFF)) {
                        int rightPosition = RightWheel.getCurrentPosition();
                        RightWheel.setTargetPosition(rightPosition);
                        canBreak = true;
                    }


                }
                RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                distance = rangeSensor.getDistance(DistanceUnit.CM);
                if (ACCEPTED_DISTANCE_FAR > distance) {
                    break;
                }

            }

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
            } else if ((topCache[0] & 0xFF) >= 7 && (topCache[0] & 0xFF) <= 11) {
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

        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }
}