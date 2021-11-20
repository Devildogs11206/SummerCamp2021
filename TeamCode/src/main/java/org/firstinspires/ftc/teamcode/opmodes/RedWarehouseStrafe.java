package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.FORWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.LiftPosition.MAX;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedWarehouseStrafe extends RedOpMode{
    @Override
    protected void execute() {
        robot.drivePower = 0.5;
        robot.lift(MAX);
        robot.drive(1,0,0, 24);
        robot.turn(1,45);
        robot.intake(FORWARD,500);
        robot.drive(0,1,-90,30);
        robot.drive(1,0,-90,36);
        robot.drive(0,1,90,20);

    }
}
