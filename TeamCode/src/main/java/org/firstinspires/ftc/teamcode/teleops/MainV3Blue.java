package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "V4 Blue Teleop")
public class MainV3Blue extends MainV3Red{

    @Override
    public List<Double> bindDriveTrain(){
        return bindBlueDriveTrain();
    }

}
