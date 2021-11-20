package org.firstinspires.ftc.teamcode.controllers;


import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.CAROUSEL;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.FORWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.NEUTRAL;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.REVERSE;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeMode.REVERSECAROUSEL;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class IntakeController extends RobotController {
    public IntakeController(OpMode opMode){
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.b) robot.intake(CAROUSEL);
        else if (gamepad2.y) robot.intake(CAROUSEL);
        else if (gamepad2.x) robot.intake(REVERSECAROUSEL);
        else if (gamepad2.right_trigger > 0.5 ) robot.intake(FORWARD);
        else if (gamepad2.left_trigger > 0.5 ) robot.intake(REVERSE);
        else robot.intake(NEUTRAL);
    }
}
