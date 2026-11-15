package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Don't Use V4 Blue Teleop")
public class MainV4Blue extends MainV4Red {

    @Override
    public List<Double> bindDriveTrain(){
        return bindBlueDriveTrain();
    }

}
