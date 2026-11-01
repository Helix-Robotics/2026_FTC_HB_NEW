package org.firstinspires.ftc.teamcode.teleops;

import java.util.List;

//@TeleOp(name = "Main V0 Blue (Don't Use!!!)")
public class MainV1Blue extends MainV1Red {

    @Override
    public List<Double> bindDriveTrain(){
        return bindBlueDriveTrain();
    }

}
