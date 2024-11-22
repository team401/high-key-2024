package frc.robot.constants;

import frc.robot.Robot;

public  class ModeConstants {
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    // Whether sim should be treated as sim or replay mode.
    // Will automatically be overridden by Mode.REAL if running on real hardware.
    public static  Mode simMode = Mode.SIM; // Mode.SIM or Mode.REPLAY

    public static  Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;
}
