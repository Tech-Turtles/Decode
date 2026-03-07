package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utility.Constants;
import static org.firstinspires.ftc.teamcode.utility.Constants.autoWait;

@Autonomous
public class PoliteFriendlyRedAutoSmallTriangle extends FriendlyAuto{
    public PoliteFriendlyRedAutoSmallTriangle(){
        alliance= Constants.Alliance.RED;
        autoWait = true;
    }
}
