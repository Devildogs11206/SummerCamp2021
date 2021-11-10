package org.firstinspires.ftc.teamcode.controllers;


import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeWheelMode.FORWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeWheelMode.NEUTRAL;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeWheelMode.REVERSE;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class IntakeController extends RobotController {
    public IntakeController(OpMode opMode){
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad1.right_trigger > 0.5 ) robot.intake(FORWARD);
        else if (gamepad1.left_trigger > 0.5 ) robot.intake(REVERSE);
        else robot.intake(NEUTRAL);
    }
}
