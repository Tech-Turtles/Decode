package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Constants;

@TeleOp
public class BlueTeleOp extends Manual{
    public BlueTeleOp(){
        alliance= Constants.Alliance.BLUE;
    }
}
