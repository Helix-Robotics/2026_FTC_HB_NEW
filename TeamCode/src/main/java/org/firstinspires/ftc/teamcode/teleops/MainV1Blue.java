package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "V1 Blue Teleop")
public class MainV1Blue extends MainV1Red{

    @Override
    public List<Double> bindDriveTrain(){
        return bindBlueDriveTrain();
    }

}
