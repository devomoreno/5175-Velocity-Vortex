/*The purpouse of this autonomous mode is to go straight, as to clear as many blocks
 out of the way of the mountain turn and then align itself into the floor goal without
 waiting 10 seconds starting from the close blue postition*/
package org.firstinspires.ftc.teamcode.pastcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import all hardware going to be used
/*public class AutonomousBlueCloseStraightFloorGoalNow extends OpMode {
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg
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

    ElapsedTime counter;


    final static double SCOOP_MIN_RANGE = 0.00;
    final static double SCOOP_MAX_RANGE = 1.00;

    double scoopDelta = 0.005;
    double leftscoopPosition;
    double rightscoopPosition;


    public AutonomousBlueCloseStraightFloorGoalNow() {
    }

  /* @Override
    public void init() {
        //scoop and trough
      /*  leftscoop = hardwareMap.servo.get("scoopleft");
        rightscoop = hardwareMap.servo.get("scoopright");
        motorRaise = hardwareMap.dcMotor.get("raisescoop");
        leftscoop.setDirection(Servo.Direction.REVERSE);
        leftscoopPosition = 0;
        rightscoopPosition = 0;
        Trough = hardwareMap.dcMotor.get("Trough");

        //Drive
        Treadleft = hardwareMap.dcMotor.get("LeftTread");
        Treadright = hardwareMap.dcMotor.get("RightTread");
        Treadright.setDirection(DcMotor.Direction.REVERSE);

        Treadleft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        Treadright.setMode(DcMotorController.RunMode.RESET_ENCODERS);


        Treadleft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        Treadright.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);


        //Zombie Arm

        LinearActuator = hardwareMap.dcMotor.get("ZLinearActuator");
        BaseExtend = hardwareMap.dcMotor.get("ZBaseExtend");
        SecondExtend = hardwareMap.dcMotor.get("ZSecondExtend");
        BaseExtend.setDirection(DcMotor.Direction.REVERSE);
        LinearActuator.setDirection(DcMotor.Direction.REVERSE);

        counter = new ElapsedTime();


        //map items here and set rules ( reference any vector baseline or basic programs)

    }

   /

        Treadleft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        Treadright.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        Treadleft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        Treadright.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);



        double currentTime = counter.time();
        int currentLeftEncoderPosition = Treadleft.getCurrentPosition();
        int currentRightEncoderPosition = Treadright.getCurrentPosition();




        if (currentTime >= 0 ) {
            Treadleft.setTargetPosition(11225);
            Treadright.setTargetPosition(11239);

            Treadleft.setPower(1);
            Treadright.setPower(1);

            Treadleft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            Treadright.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        }
        if (currentTime > 4 && currentTime<6.9f){

            Treadleft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            Treadright.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            Treadleft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            Treadright.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);


            Treadleft.setTargetPosition(1252);
            Treadright.setTargetPosition(-1137);

            Treadleft.setPower(1);
            Treadright.setPower(-1);

            Treadleft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            Treadright.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        }

        if (currentTime > 7 ){
            Treadleft.setTargetPosition(currentLeftEncoderPosition + 3380);
            Treadright.setTargetPosition(currentRightEncoderPosition + 3380);
            Treadleft.setPower(1);
            Treadright.setPower(1);
            Treadleft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            Treadright.setMode(DcMotorController.RunMode.RUN_TO_POSITION);



        }
        telemetry.addData("Left"," Encoder Target" + Treadleft.getTargetPosition());
        telemetry.addData("Left", "Left Motor: " + Treadleft.getCurrentPosition());
        telemetry.addData("Right"," Encoder Target" + Treadright.getTargetPosition());
        telemetry.addData("Right", "Right Motor: " + Treadright.getCurrentPosition());






        //set all the driver and gamepad options. this is where the program goes.
    }

    @Override
    public void stop() {
        //this is, to my knowledge all that is needed for this public void
    }
    //This is for the driving scale as far as this point it is ok without modification
}*/