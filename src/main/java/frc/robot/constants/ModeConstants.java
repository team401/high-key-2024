package frc.robot.constants;

import frc.robot.Robot;

public final class ModeConstants {
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    // Whether sim should be treated as sim or replay mode.
    // Will automatically be overridden by Mode.REAL if running on real hardware.
    public static final Mode simMode = Mode.SIM; // Mode.SIM or Mode.REPLAY

    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;
}
