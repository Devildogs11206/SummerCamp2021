package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.FORWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.REVERSE;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.REVERSECAROUSEL;
import static org.firstinspires.ftc.teamcode.internal.Robot.LiftPosition.CAROUSEL;
import static org.firstinspires.ftc.teamcode.internal.Robot.LiftPosition.LOWGOAL;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.internal.Robot;

@Autonomous
public class BlueCarousel extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drivePower = .5;
        robot.lift(LOWGOAL);
        robot.drive(1,0,0, 24);
        robot.turn(1,45);
        robot.intake(FORWARD,500);
        robot.drive(-1,0,90, 30);
        robot.drive(1,0,90);
        robot.lift(CAROUSEL);
        robot.drive(1,0, 180,13.1);
        robot.drive(0,0,180,0);
        robot.intake(REVERSECAROUSEL,5000);
        robot.drive(-1,0,180,13.1);
    }
}
