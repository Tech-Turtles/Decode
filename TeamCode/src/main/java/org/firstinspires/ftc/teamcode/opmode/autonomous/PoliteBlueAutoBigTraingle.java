package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utility.Constants;
import static org.firstinspires.ftc.teamcode.utility.Constants.autoWait;

@Autonomous
public class PoliteBlueAutoBigTraingle extends OdoAutoBigTriangle{
    public PoliteBlueAutoBigTraingle(){
        alliance = Constants.Alliance.BLUE;
        autoWait = true;

    }
}
