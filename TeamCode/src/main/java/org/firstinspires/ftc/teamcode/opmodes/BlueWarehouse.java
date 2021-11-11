package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.internal.Robot;

@Autonomous
public class BlueWarehouse extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drivePower = 0.5;
        robot.lift(Robot.LiftMode.BACKWARD,500);
        robot.drive(1,0,0, 24);
        robot.turn(1,-45);
        robot.intake(Robot.IntakeWheelMode.FORWARD,500);
        robot.drive(0,1,-90,30);
        robot.drive(-1,0,-90,36);

    }
}
