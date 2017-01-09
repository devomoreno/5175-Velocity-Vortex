package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
@TeleOp(name="Color Sensor Only", group="Test")

//import all hardware going to be used
public class TestColorVal extends OpMode {
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg0
    ColorSensor colorSensor;
    public TestColorVal(){}

    @Override
            public void init(){

        //map items here and set rules ( reference any vector baseline or basic programs)

        // bPrevState and bCurrState represent the previous and current state of the button.

        // bLedOn represents the state of the LED.

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("color sensor");

        // Set the LED in the beginning


    }
    @Override
            public void loop(){

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

}
