package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Spinner Op With Sensors", group="Opmode")

/*

import all hardware going to be used
*/
public class ContestWithSpinnerWithSensors extends OpMode{
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg

    private DcMotor LeftWheel;
    private DcMotor RightWheel;
    private DcMotor Spinner;
    private DcMotor LLAMA;

    private Servo buttonPusher;




    double FLOOR_ACCEPTED_VAL_MIN = 0;
    double FLOOR_ACCEPTED_VAL_MAX = 30;


    double servoDelta = .05;
    double ServoPosition;

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

    boolean canBreak = false;

    int TARGET_INCREASE = 1;

    public ContestWithSpinnerWithSensors(){}

    @Override
            public void init(){


        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        buttonPusher = hardwareMap.servo.get("Button Pusher");
        Spinner = hardwareMap.dcMotor.get("Spinner");
        LLAMA = hardwareMap.dcMotor.get("LLAMA");




        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN,PUSHER_MAX);
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


        FloorRightReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        FloorRightReader.write8(3, 0);


        //map items here and set rules ( reference any vector baseline or basic programs)
                //i love you and you're a cute patoot. happy programming!!--ashton

    }
    @Override
            public void loop() {
        //set all the driver and gamepad options. this is where the program goes.
        //FIXME find and replace correct data
        int LLAMAmaxPosition = 1;
        //This is the driving commands
        float left1 = gamepad1.right_stick_y;
        float right1 = gamepad1.left_stick_y;

        //for the spinner
        float motorPower=gamepad2.left_stick_y;

        //for the LLAMA
        float LLAMAPower = gamepad2.right_stick_y;


        right1 = Range.clip(right1, -1, 1);
        left1 = Range.clip(left1, -1, 1);
        motorPower = Range.clip(motorPower, -1, 1);
        LLAMAPower = Range.clip(LLAMAPower,-1,1);

        right1 = (float) scaleInput(right1);
        left1 = (float) scaleInput(left1);
        motorPower =  (float)scaleInput(motorPower);
        LLAMAPower = (float)scaleInput((LLAMAPower));

        LLAMA.setPower(Range.clip(LLAMAPower, -1,1));

        if(LLAMA.getCurrentPosition()> LLAMAmaxPosition+1){
            LLAMA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        Spinner.setPower(Range.clip(motorPower, -1, 1));
        LeftWheel.setPower(Range.clip(left1 , -1, 1));
        RightWheel.setPower(Range.clip(right1, -1, 1));


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



        if (gamepad1.y) {
            LLAMA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LLAMA.setTargetPosition(LLAMAmaxPosition);
            LLAMA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



        telemetry.addData("Right Wheel", RightWheel.getCurrentPosition());
        telemetry.addData("Left Wheel", LeftWheel.getCurrentPosition());
        telemetry.addData("LLAMA", LLAMA.getCurrentPosition());

        // This part will be an automated process to press the button, It will use teh range sensor
        //and the color sensors on the bottom to press teh button more effecitnely.


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
