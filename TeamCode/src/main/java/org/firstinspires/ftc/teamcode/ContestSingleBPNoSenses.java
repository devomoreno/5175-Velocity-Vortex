package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Contest Op: Two As One", group="Opmode")

/*

import all hardware going to be used
*/
public class ContestSingleBPNoSenses extends OpMode{
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg
    ColorSensor colorSensor;
    DcMotor LeftWheel;
    DcMotor RightWheel;
    Servo buttonPusher;





    double servoDelta = .05;
    double ServoPosition;



    public ContestSingleBPNoSenses(){}

    @Override
            public void init(){

        colorSensor = hardwareMap.colorSensor.get("color sensor");
        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        buttonPusher = hardwareMap.servo.get("Button Pusher");


        double PUSHER_MIN = 0;
        double PUSHER_MAX = 1;
        buttonPusher.scaleRange(PUSHER_MIN,PUSHER_MAX);
        buttonPusher.setDirection(Servo.Direction.REVERSE);
        //map items here and set rules ( reference any vector baseline or basic programs)




    }
    @Override
            public void loop() {
        //set all the driver and gamepad options. this is where the program goes.

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
