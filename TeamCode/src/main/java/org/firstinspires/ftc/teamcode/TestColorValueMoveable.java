package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Color on Wheels", group="Test")
@Disabled

//import all hardware going to be used
public class TestColorValueMoveable extends OpMode {
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg0
    ColorSensor colorSensor;
    DcMotor LeftWheel;
    DcMotor RightWheel;

    public TestColorValueMoveable(){}

    @Override
            public void init(){

        //map items here and set rules ( reference any vector baseline or basic programs)

        LeftWheel =hardwareMap.dcMotor.get("LeftWheel");
        RightWheel=hardwareMap.dcMotor.get("RightWheel");
        LeftWheel.setDirection(DcMotor.Direction.REVERSE);

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("color sensor");

        // Set the LED in the beginning


    }
    @Override
            public void loop(){

        float left=gamepad1.left_stick_y;
        float right=gamepad1.right_stick_y;

        right= Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        LeftWheel.setPower(left);
        RightWheel.setPower(right);

        boolean bLedOn = true;

        if(gamepad1.x) {
            bLedOn = true;
        }
        if(gamepad1.y){
            bLedOn = false;
        }


//True enables the LED and False disables the LED
        colorSensor.enableLed(bLedOn);


        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
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
