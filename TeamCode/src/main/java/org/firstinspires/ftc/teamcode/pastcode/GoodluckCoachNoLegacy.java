package org.firstinspires.ftc.teamcode.pastcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//import all hardware going to be used
public class GoodluckCoachNoLegacy extends OpMode {
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg
   //Trough and Scoop
    DcMotor Trough;
    DcMotor motorRaise;
    Servo leftscoop;
    Servo rightscoop;

    //Drive
    DcMotor Treadleft;
    DcMotor Treadright;

    //Zombie Arm

    DcMotor LinearActuator;
    DcMotor BaseExtend;
    DcMotor SecondExtend;


    final static double SCOOP_MIN_RANGE  = 0.00;
    final static double SCOOP_MAX_RANGE  = 1.00;

    double scoopDelta=0.005;
    double leftscoopPosition;
    double rightscoopPosition;
    double generalPower;

    public GoodluckCoachNoLegacy(){}

    @Override
            public void init(){
        //scoop and trough
        leftscoop=hardwareMap.servo.get("scoopleft");
        rightscoop=hardwareMap.servo.get("scoopright");
        motorRaise=hardwareMap.dcMotor.get("raisescoop");
        leftscoop.setDirection(Servo.Direction.REVERSE);
        leftscoopPosition=0;
        rightscoopPosition=0;
        Trough = hardwareMap.dcMotor.get("Trough");

        //Drive
        Treadleft=hardwareMap.dcMotor.get("LeftTread");
        Treadright=hardwareMap.dcMotor.get("RightTread");
        Treadleft.setDirection(DcMotor.Direction.REVERSE);

        //Zombie Arm

        LinearActuator = hardwareMap.dcMotor.get("ZLinearActuator");
        BaseExtend = hardwareMap.dcMotor.get("ZBaseExtend");
        SecondExtend = hardwareMap.dcMotor.get("ZSecondExtend");
        BaseExtend.setDirection(DcMotor.Direction.REVERSE);
        LinearActuator.setDirection(DcMotor.Direction.REVERSE);


        //map items here and set rules ( reference any vector baseline or basic programs)

    }


    @Override
            public void loop(){


        if (gamepad1.left_bumper) {
            Trough.setPower(1);
        }
        if (gamepad1.right_bumper) {
            Trough.setPower(-1);
        }
        if (!gamepad1.left_bumper && !gamepad1.right_bumper){
            Trough.setPower(0);
        }



        //Scoop



        if (gamepad2.right_bumper) {
            leftscoopPosition += scoopDelta;
            rightscoopPosition += scoopDelta;
        }
        if (gamepad2.left_bumper){
            leftscoopPosition -= scoopDelta;
            rightscoopPosition -= scoopDelta;

        }

        //Zombie Arm





        if(gamepad2.left_stick_y==0){
            LinearActuator.setPower(0);
        }

        if(gamepad2.left_trigger != 0){
            BaseExtend.setPower(-.5);
            SecondExtend.setPower(-.5);
        }
        if(gamepad2.right_trigger != 0){
            BaseExtend.setPower (.5);
            SecondExtend.setPower (.5);
        }

        if(gamepad2.left_trigger == 0 && gamepad2.right_trigger ==0){
            BaseExtend.setPower (0);
            SecondExtend.setPower (0);
        }

        if(gamepad2.right_stick_y>0){
            motorRaise.setPower(.5);

        }

        if (gamepad2.right_stick_y<0){
            motorRaise.setPower(-.5);
        }
        if (gamepad2.right_stick_y == 0){
            motorRaise.setPower(0);
        }

        //Trough


      //  if (!gamepad1.left_bumper && !gamepad1.right_bumper){
        //    Trough.setPower(0);
        //}


        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;
        float motorPower = gamepad2.left_stick_y;





        right= Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        generalPower = Range.clip(generalPower, -1, 1);
        motorPower = Range.clip(motorPower, -1, 1);


        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);
        generalPower=(float)scaleInput(generalPower);
        motorPower = (float) scaleInput(motorPower);



        Treadleft.setPower(left);
        Treadright.setPower(right);
        LinearActuator.setPower(motorPower*.75);





        leftscoopPosition = Range.clip(leftscoopPosition, SCOOP_MIN_RANGE, SCOOP_MAX_RANGE);
        rightscoopPosition = Range.clip(rightscoopPosition, SCOOP_MIN_RANGE, SCOOP_MAX_RANGE);

        // write position values to the wrist and claw servo
        leftscoop.setPosition(leftscoopPosition);
        rightscoop.setPosition(rightscoopPosition);

        //set all the driver and gamepad options. this is where the program goes.
    }
    @Override
        public void stop(){

        Trough.setPower(0);
        motorRaise.setPower(0);

        //Drive
         Treadleft.setPower(0);
        Treadright.setPower(0);

        //Zombie Arm

         LinearActuator.setPower(0);
        BaseExtend.setPower(0);
         SecondExtend.setPower(0);

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