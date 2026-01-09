package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utility.Constants;
import static org.firstinspires.ftc.teamcode.utility.Constants.autoWait;

@Autonomous
public class PoliteRedAutoSmallTraingle extends OdoAuto{
    public PoliteRedAutoSmallTraingle(){
        alliance= Constants.Alliance.RED;
        autoWait = true;
    }
}
