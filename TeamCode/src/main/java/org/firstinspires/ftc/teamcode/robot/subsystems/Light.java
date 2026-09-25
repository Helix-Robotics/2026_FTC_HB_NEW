package org.firstinspires.ftc.teamcode.robot.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Light {
    public Servo light;

    public final double red = 0.277;
    public final double orange = 0.333;
    public final double yellow = 0.388;
    public final double lgreen = 0.444;
    public final double green = 0.500;
    public final double azure = 0.555;
    public final double blue = 0.611;
    public final double indigo = 0.666;
    public final double purple = 0.722;
    public final double pink = 0.820;
    public final double white = 1.000;


    public Light(HardwareMap hw) {
        light = hw.get(Servo.class, "light");
    }

    public void setColour(double colour) {light.setPosition(colour);}

    public void red() {setColour(red);}
    public void orange() {setColour(orange);}
    public void yellow() {setColour(yellow);}
    public void lgreen() {setColour(lgreen);}
    public void green() {setColour(green);}
    public void azure() {setColour(azure);}
    public void blue() {setColour(blue);}
    public void indigo() {setColour(indigo);}
    public void purple() {setColour(purple);}
    public void pink() {setColour(pink);}
    public void white() {setColour(white);}



    public class RedLight implements Action {

        //private boolean hold = false;


        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            red();
            return false;
        }

    }

    public Action redlightAction() {return new RedLight(); }



}
