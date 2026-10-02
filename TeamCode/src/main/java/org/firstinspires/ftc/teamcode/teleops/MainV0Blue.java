package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Main V0 Blue (Don't Use!!!)")
public class MainV0Blue extends MainV0Red{

    @Override
    public List<Double> bindDriveTrain(){
        return bindBlueDriveTrain();
    }

}
