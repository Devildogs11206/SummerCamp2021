package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.internal.Alliance.BLUE;

import org.firstinspires.ftc.teamcode.internal.Alliance;

public abstract class BlueOpMode extends TeleOpMode {
    @Override
    protected Alliance getAlliance() {
        return BLUE;
    }
}
