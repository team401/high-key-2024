package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.AlignTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ShootWithGamepad extends Command {

    private BooleanSupplier driverShoot;
    private BooleanSupplier masherShoot;
    private BooleanSupplier masherForceShoot;
    private BooleanSupplier warmup;
    private BooleanSupplier reverseIntake;
    private BooleanSupplier intake;

    private ScoringSubsystem scoring;

    private Supplier<AlignTarget> getDriveMode;

    public ShootWithGamepad(
            BooleanSupplier driverShoot,
            BooleanSupplier masherShoot,
            BooleanSupplier masherForceShoot,
            BooleanSupplier warmup,
            BooleanSupplier reverseIntake,
            BooleanSupplier intake,
            ScoringSubsystem scoring,
            Supplier<AlignTarget> getDriveMode) {
        this.driverShoot = driverShoot;
        this.masherShoot = masherShoot;
        this.masherForceShoot = masherForceShoot;
        this.warmup = warmup;
        this.reverseIntake = reverseIntake;
        this.intake = intake;

        this.scoring = scoring;
        this.getDriveMode = getDriveMode;

        addRequirements(scoring);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (DriverStation.isAutonomous() || DriverStation.isTest()) {
            return;
        }

        if (driverShoot.getAsBoolean() || masherShoot.getAsBoolean()) {
            warmupSwitch();
            if (getDriveMode.get() != AlignTarget.SOURCE
                    && getDriveMode.get() != AlignTarget.NONE) {
                boolean force = masherForceShoot.getAsBoolean();
                scoring.setOverrideShoot(force);

                if (driverShoot.getAsBoolean() || masherShoot.getAsBoolean() || force) {
                    scoring.setAction(ScoringAction.SHOOT);
                }
            }
        } else if (masherForceShoot.getAsBoolean()) {
            scoring.setAction(ScoringAction.SHOOT);
        } else if (warmup.getAsBoolean()) {
            warmupSwitch();
        } else if (reverseIntake.getAsBoolean()) {
            scoring.setAction(ScoringAction.SPIT);
        } else if (intake.getAsBoolean()) {
            scoring.setAction(ScoringAction.INTAKE);
        } else {
            scoring.setAction(ScoringAction.WAIT);
        }
    }

    public void warmupSwitch() {
        switch (getDriveMode.get()) {
            case NONE:
                scoring.setAction(ScoringAction.WAIT);
                break;
            case SPEAKER:
                scoring.setAction(ScoringAction.AIM);
                break;
            case AMP:
                scoring.setAction(ScoringAction.AMP_AIM);
                break;
            case SOURCE:
                scoring.setAction(ScoringAction.SOURCE_INTAKE);
                break;
            case ENDGAME:
                scoring.setAction(ScoringAction.ENDGAME);
                break;
        }
    }
}