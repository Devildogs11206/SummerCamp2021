package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class ArmController extends RobotController {
    private static final double INCREMENT = 0.01;

    public ArmController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad1.left_trigger > 0) robot.armLeft(-INCREMENT);
        else if (gamepad1.left_bumper) robot.armLeft(+INCREMENT);

        if (gamepad1.right_trigger > 0) robot.armRight(+INCREMENT);
        else if (gamepad1.right_bumper) robot.armRight(-INCREMENT);
    }
}