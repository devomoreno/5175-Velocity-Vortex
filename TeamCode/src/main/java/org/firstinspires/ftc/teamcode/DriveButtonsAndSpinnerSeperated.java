package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Last modified 12/10/2016
 * by William Lord
 */

@TeleOp(name = "As before + Spinning Zipties", group = "OpMode")
@Disabled
//import all hardware going to be used
public class DriveButtonsAndSpinnerSeperated extends OpMode {
    //name DcMotors and for purpose of the program
    //ex:  DcMotor Greg
    DcMotor LeftWheel;
    DcMotor RightWheel;
    DcMotor ZiptieSpinner;
    Servo LeftServo;
    Servo RightServo;

    final static double SERVO_MIN_RANGE  = 0.00;
    final static double SERVO_MAX_RANGE  = 1.00;

    double servoDelta=0.005;
    double LeftServoPosition;
    double RightServoPosition;


    public DriveButtonsAndSpinnerSeperated(){}

    @Override
    public void init(){
        LeftWheel =hardwareMap.dcMotor.get("LeftWheel");
        RightWheel=hardwareMap.dcMotor.get("RightWheel");
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        ZiptieSpinner=hardwareMap.dcMotor.get("Spinner");

        LeftServo= hardwareMap.servo.get("LeftServo");
        RightServo = hardwareMap.servo.get("RightServo");


        LeftServoPosition = 1;
        RightServoPosition = 0;


        //map items here and set rules ( reference any vector baseline or basic programs)

    }
    @Override
    public void loop(){
        float left1=gamepad1.left_stick_y;
        float right1=gamepad1.right_stick_y;
        float right2=gamepad2.right_stick_y;

        right1= Range.clip(right1, -1, 1);
        left1 = Range.clip(left1, -1, 1);
        right2= Range.clip(right2, -1, 1);

        right1 = (float)scaleInput(right1);
        left1 =  (float)scaleInput(left1);
        right2 = (float)scaleInput(right2);

        LeftWheel.setPower(left1);
        RightWheel.setPower(right1);
        ZiptieSpinner.setPower(right2);

        if(gamepad1.left_bumper){
            if(LeftServoPosition != 0){
                LeftServoPosition -= servoDelta;
            }

        }
        if (gamepad1.right_bumper){
            if(RightServoPosition != SERVO_MAX_RANGE){
                RightServoPosition += servoDelta;
            }
        }
        
        if(gamepad1.right_trigger>0){
            if(RightServoPosition != 0){
                RightServoPosition -= servoDelta;
            }
        }
        
        
        if(gamepad1.left_trigger>0){
            if(LeftServoPosition != SERVO_MAX_RANGE){
                LeftServoPosition += servoDelta;
            }
        }

        LeftServoPosition = Range.clip(LeftServoPosition, SERVO_MIN_RANGE, SERVO_MAX_RANGE);
        RightServoPosition = Range.clip(RightServoPosition, SERVO_MIN_RANGE, SERVO_MAX_RANGE);

        // write position values to the wrist and claw servo
        LeftServo.setPosition(LeftServoPosition);
        RightServo.setPosition(RightServoPosition);
    //set all the driver and gamepad options. this is where the program goes.
    }

    @Override
    public void stop(){
        //this is, to my knowledge, all that is needed for this function
    }
    //This is for the driving scale as far as this point it is ok without modification
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.00, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
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
