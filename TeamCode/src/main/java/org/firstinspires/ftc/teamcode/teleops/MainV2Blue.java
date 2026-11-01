package org.firstinspires.ftc.teamcode.teleops;

import java.util.List;

//@TeleOp(name = "V1 Blue Teleop")
public class MainV2Blue extends MainV2Red {

    @Override
    public List<Double> bindDriveTrain(){
        return bindBlueDriveTrain();
    }

}
