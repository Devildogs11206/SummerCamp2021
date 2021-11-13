package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.internal.Robot.LiftMode.BACKWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.LiftMode.FORWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.LiftMode.STOPPED;

import org.firstinspires.ftc.teamcode.internal.Robot;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class LiftController extends RobotController {
    public LiftController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.y) robot.lift(Robot.LiftPosition.CAROUSEL);
        else if (gamepad2.right_bumper) robot.lift(FORWARD);
        else if (gamepad2.left_bumper) robot.lift(BACKWARD);
        else robot.lift(STOPPED);
    }
}