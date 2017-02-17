package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Test Encoder Line Shimmy", group = "Sensors")

public class TestEncoderLineShimmy extends LinearOpMode {
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

        double distance = 0;
        double ACCEPTED_DISTANCE_CLOSE = 11 ;
        double ACCEPTED_DISTANCE_FAR = 14 ;
        double FLOOR_ACCEPTED_VAL_MIN = 1;
        double FLOOR_ACCEPTED_VAL_MAX = 30;
        double ALLOWED_FROM_WALL_MIN = 11;
        double ALLOWED_FROM_WALL_MAX = 16;

        boolean canBreak = false;

        int REVERSE_AMOUNT = 50;
        int TARGET_INCREASE = 1;

        LeftWheel.setPower(.3);
        RightWheel.setPower(.3);

      //This assumes the robot is already on the white line, it is only meant for getting the
        //robot closer to the beacon

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

                // back up

                int leftPosition = LeftWheel.getCurrentPosition();
                int rightPosition = RightWheel.getCurrentPosition();

                int leftInitialReverse = leftPosition - REVERSE_AMOUNT;
                int rightInitialReverse = rightPosition - REVERSE_AMOUNT;

                LeftWheel.setTargetPosition(leftInitialReverse);
                RightWheel.setTargetPosition(rightInitialReverse);

                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                for(int target = 0; (!canBreak) && opModeIsActive(); target += TARGET_INCREASE) {
                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                        int leftPosition2 = LeftWheel.getCurrentPosition();
                        LeftWheel.setTargetPosition(leftPosition2);
                        canBreak = true;
                    } else {
                        LeftWheel.setTargetPosition(target);
                    }


                    RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (LeftWheel.isBusy() && opModeIsActive()) {
                        leftFloorCache = FloorLeftReader.read(0x04, 1);

                        if (FLOOR_ACCEPTED_VAL_MIN <= (leftFloorCache[0] & 0xFF)) {
                            leftPosition = LeftWheel.getCurrentPosition();
                            LeftWheel.setTargetPosition(leftPosition);
                            LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            canBreak = true;

                        }

                    }
                }
                LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                canBreak = false;

                RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                for(int target = 0; (!canBreak) && opModeIsActive(); target += TARGET_INCREASE){
                    leftFloorCache = FloorLeftReader.read(0x04, 1);
                    topCache = beaconFlagReader.read(0x04, 1);
                    rightFloorCache = FloorRightReader.read(0x04, 1);

                    if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {
                        int rightPosition2 = RightWheel.getCurrentPosition();
                        RightWheel.setTargetPosition(rightPosition2);
                        canBreak = true;
                    }
                    else{
                        RightWheel.setTargetPosition(target);
                    }


                    RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (RightWheel.isBusy() && opModeIsActive()) {
                        rightFloorCache = FloorRightReader.read(0x04, 1);

                        if (FLOOR_ACCEPTED_VAL_MIN <= (rightFloorCache[0] & 0xFF)) {
                            rightPosition = RightWheel.getCurrentPosition();
                            RightWheel.setTargetPosition(rightPosition);
                            RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            canBreak = true;

                        }

                    }
                }
                RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                distance = rangeSensor.getDistance(DistanceUnit.CM);
                if (ACCEPTED_DISTANCE_FAR > distance) {
                    break;
                }
                distance = rangeSensor.getDistance(DistanceUnit.CM);
            }


        }

        telemetry.addLine("Distance looks good, looking for button...");
        telemetry.update();



    }

}