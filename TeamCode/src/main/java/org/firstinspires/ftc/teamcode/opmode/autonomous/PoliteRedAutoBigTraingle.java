package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utility.Constants;
import static org.firstinspires.ftc.teamcode.utility.Constants.autoWait;

@Autonomous
public class PoliteRedAutoBigTraingle extends OdoAutoBigTriangle{
    public PoliteRedAutoBigTraingle(){
        alliance= Constants.Alliance.RED;
        autoWait = true;

    }
}
