package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.utility.Constants.gateClosed;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class Testtransfer extends OpMode {
    protected CRServo Transfer1;
    protected CRServo Transfer2;
    boolean buttonPressed = false;

    @Override
    public void init() {
        Transfer1 = hardwareMap.get(CRServo.class,"Transfer1");
        Transfer2 = hardwareMap.get(CRServo.class,"Transfer2");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            Transfer1.setPower(1);
            Transfer2.setPower(-1);
        }else {
            Transfer1.setPower(0);
            Transfer2.setPower(0);
        }

        if(gamepad1.b){
            Transfer1.setPower(-1);
            Transfer2.setPower(1);
        }else {
            Transfer1.setPower(0);
            Transfer2.setPower(0);
        }
        if(gamepad1.x){
            buttonPressed = true;
        } else{
            buttonPressed = false;
        }


        while(buttonPressed == true){
            Transfer1.setPower(-1);
            Transfer2.setPower(1);
        }

    }
}
