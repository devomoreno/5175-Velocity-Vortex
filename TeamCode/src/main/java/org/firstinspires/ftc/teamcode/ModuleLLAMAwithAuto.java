package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="LLAMA with Auto", group="Module")

//import all hardware going to be used
public class ModuleLLAMAwithAuto extends OpMode {
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg
    private DcMotor LLAMA;
    public ModuleLLAMAwithAuto(){}

    @Override
            public void init(){
        LLAMA = hardwareMap.dcMotor.get("LLAMA");



        //map items here and set rules ( reference any vector baseline or basic programs)

    }
    @Override
            public void loop(){

        //TODO use actual rotation numbers as definded by the rotation tester
        int maxPosition = 1730;
        int LLAMAmin = 0;
        if (LLAMA.getCurrentPosition()>LLAMAmin ){
        float LLAMAPower = gamepad2.right_stick_y;
        LLAMAPower = Range.clip(LLAMAPower,-1,1);
        LLAMAPower = (float)scaleInput((LLAMAPower));
        LLAMA.setPower(Range.clip(LLAMAPower, -1,1));

        if(LLAMA.getCurrentPosition() > maxPosition+10){
            LLAMA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LLAMA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(LLAMA.getCurrentPosition()<LLAMAmin){
            LLAMA.setPower(0);
        }

        telemetry.addData("LLAMA Value", LLAMA.getCurrentPosition());

        if((gamepad2.a)) {
            LLAMA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LLAMA.setTargetPosition(maxPosition);
            LLAMA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        }
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
