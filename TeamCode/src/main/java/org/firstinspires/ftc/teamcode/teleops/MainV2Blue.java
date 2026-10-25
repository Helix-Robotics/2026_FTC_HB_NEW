package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "V3 Blue Teleop")
public class MainV2Blue extends MainV2Red{

    @Override
    public List<Double> bindDriveTrain(){
        return bindBlueDriveTrain();
    }

}
