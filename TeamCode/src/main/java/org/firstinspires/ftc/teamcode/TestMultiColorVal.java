package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

@TeleOp(name="Multi Color Sensors", group="Test")

//import all hardware going to be used
public class TestMultiColorVal extends OpMode {
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg0
    ColorSensor colorSensor;
    ColorSensor FloorSensorRight;
    ColorSensor FloorSensorLeft;
    public TestMultiColorVal(){}

    @Override
            public void init(){

        //map items here and set rules ( reference any vector baseline or basic programs)

        // bPrevState and bCurrState represent the previous and current state of the button.

        // bLedOn represents the state of the LED.

        // get a reference to our ColorSensor object.
        FloorSensorRight = hardwareMap.colorSensor.get("Right color sensor");
        FloorSensorLeft = hardwareMap.colorSensor.get("Left color sensor");
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        I2cAddr colorAddr = I2cAddr.create8bit(0x3c);
        I2cAddr floorRightAddr = I2cAddr.create8bit(0x3a);
        I2cAddr floorLeftAddr = I2cAddr.create8bit(0x3e);

        colorSensor.setI2cAddress(colorAddr);
        FloorSensorLeft.setI2cAddress(floorLeftAddr);
        FloorSensorRight.setI2cAddress(floorRightAddr);

        // Set the LED in the beginning


    }
    @Override
            public void loop(){


        FloorSensorLeft.enableLed(false);
        FloorSensorRight.enableLed(false);
        colorSensor.enableLed(false);

        if(gamepad1.y){
            colorSensor.enableLed(true);
        }
        if(gamepad1.b){
            FloorSensorRight.enableLed(true);
        }
        if(gamepad1.x){
            FloorSensorLeft.enableLed(true);

        }


//True enables the LED and False disables the LED






        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Right Clear", FloorSensorRight.alpha());
        telemetry.addData("Right Red  ", FloorSensorRight.red());
        telemetry.addData("Right Green", FloorSensorRight.green());
        telemetry.addData("Right Blue ", FloorSensorRight.blue());
        telemetry.addData("Left Clear", FloorSensorLeft.alpha());
        telemetry.addData("Left Red  ", FloorSensorLeft.red());
        telemetry.addData("Left Green",FloorSensorLeft.green());
        telemetry.addData("Left Blue ", FloorSensorLeft.blue());

    //set all the driver and gamepad options. this is where the program goes.
    }
    @Override
        public void stop(){
        //this is, to my knowledge all that is needed for this public void
    }
    //This is for the driving scale as far as this point it is ok without modification

}
