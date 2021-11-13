package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.FORWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.LiftMode.BACKWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.LiftPosition.LOWGOAL;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueWarehouse extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drivePower = 0.5;
        robot.lift(LOWGOAL);
        robot.drive(1,0,0, 24);
        robot.turn(1,-45);
        robot.intake(FORWARD,500);
        robot.drive(0,-1,90,30);
        robot.drive(1,0,90,36);
    }
}
