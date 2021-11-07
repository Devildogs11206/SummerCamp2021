package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueCarousel extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drivePower = 0.25;
        robot.drive(1,0,0, 22);
        robot.turn(1,-45);
        // TODO: Add code for delivering the pre-loaded freight
        robot.turn(1,-45);
        robot.drive(-1,0,0, 35);
    }
}