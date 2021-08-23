package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class GateController extends RobotController {
    public GateController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad1.y) robot.moveGate(+0.1);
        else if (gamepad1.a) robot.moveGate(-0.1);
        else robot.moveGate(0);
    }
}