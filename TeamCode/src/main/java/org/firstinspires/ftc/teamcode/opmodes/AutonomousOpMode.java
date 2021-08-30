package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonomousOpMode extends TeleOpMode {
    @Override
    protected void execute() {
        robot.drivePower = 0.2;
        robot.driveInches(1, 0, 0, 12);
        robot.driveSeconds(1, 0, 0, 3);
    }
}