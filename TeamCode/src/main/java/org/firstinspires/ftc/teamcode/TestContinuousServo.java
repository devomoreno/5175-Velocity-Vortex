package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test Cont Servo", group = "Module")
//import all hardware going to be used
public class TestContinuousServo extends OpMode {
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg
    CRServo contiServo;

    public TestContinuousServo(){}

    @Override
            public void init(){
        contiServo = hardwareMap.crservo.get("cServo");


        //map itemshere and set rules ( reference any vector baseline or basic programs)

    }
    @Override
            public void loop(){

        float servoRotPower=gamepad1.left_stick_y;


         servoRotPower= Range.clip(servoRotPower, -1, 1);



        servoRotPower =  (float)scaleInput(servoRotPower);


         contiServo.setPower(servoRotPower);

        if(gamepad1.y){
           contiServo.setPower(0);
        }

    //set all the driver and gamepad options. this is where the program goes.
    }
    @Override
        public void stop(){
        //this is, to my knowledge all that is needed for this public void
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
