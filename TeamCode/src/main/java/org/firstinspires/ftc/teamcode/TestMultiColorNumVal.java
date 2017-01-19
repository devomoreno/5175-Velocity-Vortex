package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Multi Color Number Sensors", group="Test")

//import all hardware going to be used
public class TestMultiColorNumVal extends OpMode {
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg0


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



    public TestMultiColorNumVal(){}

    @Override
            public void init(){

        //map items here and set rules ( reference any vector baseline or basic programs)

        // bPrevState and bCurrState represent the previous and current state of the button.

        // bLedOn represents the state of the LED.

        // get a reference to our ColorSensor object.


        beaconFlagSensor = hardwareMap.i2cDevice.get("color sensor");
        FloorLeft = hardwareMap.i2cDevice.get("Left color sensor");
        FloorRight = hardwareMap.i2cDevice.get("Right color sensor");

        beaconFlagReader= new I2cDeviceSynchImpl(beaconFlagSensor, I2cAddr.create8bit(0x3c), false);
        FloorLeftReader = new I2cDeviceSynchImpl(FloorLeft, I2cAddr.create8bit(0x3e), false);
        FloorRightReader = new I2cDeviceSynchImpl(FloorRight, I2cAddr.create8bit(0x3a), false);

        beaconFlagReader.engage();
        FloorLeftReader.engage();
        FloorRightReader.engage();



        // Set the LED in the beginning


    }
    @Override
    public void start() {

        if(beaconFlagLEDState){
           beaconFlagReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        }
        else{
           beaconFlagReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }


        if(FloorLeftLEDState){
            FloorLeftReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
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
    }

    @Override
            public void loop(){


        //The below two if() statements ensure that the mode of the color sensor is changed only once each time the touch sensor is pressed.
        //The mode of the color sensor is saved to the sensor's long term memory. Just like flash drives, the long term memory has a life time in the 10s or 100s of thousands of cycles.
        //This seems like a lot but if your program wrote to the long term memory every time though the main loop, it would shorten the life of your sensor.

        if (!XtouchState && gamepad1.x) {  //If the touch sensor is just now being pressed (was not pressed last time through the loop but now is)
            XtouchState = true;                   //Change touch state to true because the touch sensor is now pressed
            FloorLeftLEDState = !FloorLeftLEDState;                //Change the LEDState to the opposite of what it was
            if(FloorLeftLEDState){
               FloorLeftReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
                //Set the mode of the color sensor using LEDState
            }
            else{
                FloorLeftReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
                   //Set the mode of the color sensor using LEDState
            }

        }
        if (!gamepad1.x) {                //If the touch sensor is now pressed
            XtouchState = false;                  //Set the touchState to false to indicate that the touch sensor was released
        }

        if (!YtouchState && gamepad1.y) {  //If the touch sensor is just now being pressed (was not pressed last time through the loop but now is)
            YtouchState = true;                   //Change touch state to true because the touch sensor is now pressed
           beaconFlagLEDState = !beaconFlagLEDState;                //Change the LEDState to the opposite of what it was
            if(beaconFlagLEDState){
                beaconFlagReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
                //Set the mode of the color sensor using LEDState
            }
            else{
                beaconFlagReader.write8(3, 1);    //Set the mode of the color sensor using LEDState
                //Set the mode of the color sensor using LEDState
            }

        }
        if (!gamepad1.y) {                //If the touch sensor is now pressed
            YtouchState = false;                  //Set the touchState to false to indicate that the touch sensor was released
        }

        if (!BtouchState && gamepad1.b) {  //If the touch sensor is just now being pressed (was not pressed last time through the loop but now is)
            BtouchState = true;                   //Change touch state to true because the touch sensor is now pressed
            FloorRightLEDState = !FloorRightLEDState;                //Change the LEDState to the opposite of what it was
            if(FloorRightLEDState){
                FloorRightReader.write8(3, 0);    //Set the mode of the color sensor using LEDState
                //Set the mode of the color sensor using LEDState
            }
            else{
                FloorRightReader.write8(3, 1);

                //Set the mode of the color sensor using LEDState
                //Set the mode of the color sensor using LEDState
            }

        }
        if (!gamepad1.b) {                //If the touch sensor is now pressed
            BtouchState = false;                  //Set the touchState to false to indicate that the touch sensor was released
        }


        colorFloorLeftCache = FloorLeftReader.read(0x04, 1);
       colorFlagcache = beaconFlagReader.read(0x04, 1);
        colorFloorRightCache = FloorRightReader.read(0x04, 1);

        //display values
        telemetry.addData("1 #A", colorFloorLeftCache[0] & 0xFF);
        telemetry.addData("2 #C", colorFlagcache[0] & 0xFF);
        telemetry.addData("3 #E", colorFloorRightCache[0] & 0xFF);

        telemetry.addData("4 A", FloorLeftReader.getI2cAddress().get8Bit());
        telemetry.addData("5 C", beaconFlagReader.getI2cAddress().get8Bit());
        telemetry.addData("6 E", FloorRightReader.getI2cAddress().get8Bit());

    //set all the driver and gamepad options. this is where the program goes.
    }
    @Override
        public void stop(){
        //this is, to my knowledge all that is needed for this public void
    }
    //This is for the driving scale as far as this point it is ok without modification

}
