package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedCarousel extends RedOpMode {
    @Override
    protected void execute() {
        robot.drivePower = 0.5;
        robot.drive(1,0,0, 26);
        robot.turn(1,-45);
        // TODO: Add code for delivering the pre-loaded freight
        robot.drive(-1,0,-90, 35);
    }
}
