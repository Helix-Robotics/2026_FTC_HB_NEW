package org.firstinspires.ftc.teamcode.teleops;

import java.util.List;

//@TeleOp(name = "V4 Blue Teleop")
public class MainV4Blue extends MainV4Red {

    @Override
    public List<Double> bindDriveTrain(){
        return bindBlueDriveTrain();
    }

}
