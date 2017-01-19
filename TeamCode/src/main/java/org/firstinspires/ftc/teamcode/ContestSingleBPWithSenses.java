package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Sensor Op", group="Opmode")

/*

import all hardware going to be used
*/
public class ContestSingleBPWithSenses extends OpMode{
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg

    DcMotor LeftWheel;
    DcMotor RightWheel;
    ModernRoboticsI2cRangeSensor rangeSensor;
    Servo buttonPusher;

    double FLOOR_ACCEPTED_VAL_MIN = 0;
    double FLOOR_ACCEPTED_VAL_MAX = 30;

    double servoDelta = .005;
    double ServoPosition;

    byte[] colorFlagcache;
    byte[] colorFloorRightCache;
    byte[] colorFloorLeftCache;

    I2cDevice beaconFlagSensor;
    I2cDevice FloorRight;
    I2cDevice FloorLeft;

    I2cDeviceSynch beaconFlagReader;
    I2cDeviceSynch FloorRightReader;
    I2cDeviceSynch FloorLeftReader;

    boolean YtouchState = false;  //Tracks the last known state of the A button
    boolean BtouchState = false;
    boolean XtouchState = false;

    boolean beaconFlagLEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false
    boolean FloorRightLEDState = true;
    boolean FloorLeftLEDState = true;




    public ContestSingleBPWithSenses(){}

    @Override
            public void init(){
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        buttonPusher = hardwareMap.servo.get("Button Pusher");
        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN,PUSHER_MAX);


        beaconFlagSensor = hardwareMap.i2cDevice.get("color sensor");
        FloorLeft = hardwareMap.i2cDevice.get("Left color sensor");
        FloorRight = hardwareMap.i2cDevice.get("Right color sensor");

        beaconFlagReader= new I2cDeviceSynchImpl(beaconFlagSensor, I2cAddr.create8bit(0x3c), false);
        FloorLeftReader = new I2cDeviceSynchImpl(FloorLeft, I2cAddr.create8bit(0x3e), false);
        FloorRightReader = new I2cDeviceSynchImpl(FloorRight, I2cAddr.create8bit(0x3a), false);

        beaconFlagReader.engage();
        FloorLeftReader.engage();
        FloorRightReader.engage();


        FloorRightReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        FloorRightReader.write8(3, 0);


        //map items here and set rules ( reference any vector baseline or basic programs)




    }
    @Override
            public void loop() {
        //set all the driver and gamepad options. this is where the program goes.
        colorFloorLeftCache = FloorLeftReader.read(0x04, 1);
        colorFlagcache = beaconFlagReader.read(0x04, 1);
        colorFloorRightCache = FloorRightReader.read(0x04, 1);

        //This is the driving commands
        float left1 = gamepad1.right_stick_y;
        float right1 = gamepad1.left_stick_y;
        float left2 = gamepad2.right_stick_y;
        float right2 = gamepad2.left_stick_y;


        right1 = Range.clip(right1, -1, 1);
        left1 = Range.clip(left1, -1, 1);
        right2 = Range.clip(right2, -1, 1);
        left2 = Range.clip(left2, -1, 1);

        right1 = (float) scaleInput(right1);
        left1 = (float) scaleInput(left1);
        right2 = (float) scaleInput(right2);
        left2 = (float) scaleInput(left2);

        LeftWheel.setPower(Range.clip(left1 + left2, -1, 1)); /* Currently takes the sum of both
         controller inputs and clips them to -1 and 1. If one joystick is pushed all the way
         forward/back, the other controller will not affect motion. Otherwise, both controllers are
         taken into account. */
       RightWheel.setPower(Range.clip(right1 + right2, -1, 1));


        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            if (ServoPosition != 1) {
                ServoPosition += servoDelta;
            }
        }

        if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
            if (ServoPosition != 0) {
                ServoPosition -= servoDelta;
            }
        }


        buttonPusher.setPosition(ServoPosition);

        // This part will be an automated process to press the button, It will use teh range sensor
        //and the color sensors on the bottom to press teh button more effecitnely.

        if (gamepad1.a || gamepad2.a) {
            colorFloorLeftCache = FloorLeftReader.read(0x04, 1);
           colorFloorRightCache = FloorRightReader.read(0x04, 1);
            while(!((FLOOR_ACCEPTED_VAL_MIN <= (colorFloorLeftCache[0] & 0xFF)||
                    FLOOR_ACCEPTED_VAL_MIN <= (colorFloorRightCache[0] & 0xFF)))) {

                colorFloorLeftCache = FloorLeftReader.read(0x04, 1);
                colorFloorRightCache = FloorRightReader.read(0x04, 1);


                LeftWheel.setPower(.3);
                if (FLOOR_ACCEPTED_VAL_MIN <= (colorFloorLeftCache[0] & 0xFF)) {
                    LeftWheel.setPower(0);
                }
                RightWheel.setPower(.3);
                if (FLOOR_ACCEPTED_VAL_MIN <= (colorFloorRightCache[0] & 0xFF)) {
                    RightWheel.setPower(0);
                }
                if (((RightWheel.getPower())==0)  && ((LeftWheel.getPower()) ==0)){
                    break;
                }
            }

        }

    }
    @Override
        public void stop(){
        //this is, to my knowledge all that is needed for this public void
    }
    //This is for the driving scale as far as this point it is ok without modification
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
