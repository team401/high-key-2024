package frc.robot.subsystems.scoring;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.ConversionConstants;
import frc.robot.constants.ScoringConstants;
import frc.robot.constants.SimConstants;

public class ShooterIOSim implements ShooterIO {
    // TODO: Tune this later
    private final FlywheelSim shooterLeftSim =
            new FlywheelSim(DCMotor.getKrakenX60(1), 0.5, 0.0010639061);
    private final FlywheelSim shooterRightSim =
            new FlywheelSim(DCMotor.getKrakenX60(1), 0.5, 0.0010639061);

    private final PIDController shooterLeftController =
            new PIDController(
                    ScoringConstants.shooterkP,
                    ScoringConstants.shooterkI,
                    ScoringConstants.shooterkD);
    private final PIDController shooterRightController =
            new PIDController(
                    ScoringConstants.shooterkP,
                    ScoringConstants.shooterkI,
                    ScoringConstants.shooterkD);

    private final SimpleMotorFeedforward shooterFeedforward =
            new SimpleMotorFeedforward(
                    ScoringConstants.shooterkS,
                    ScoringConstants.shooterkV,
                    ScoringConstants.shooterkA);

    private boolean override = false;

    double shooterLeftGoalVelRPM = 0.0;
    double shooterLeftAppliedVolts = 0.0;

    double shooterRightGoalVelRPM = 0.0;
    double shooterRightAppliedVolts = 0.0;

    double kickerVolts = 0.0;

    @Override
    public void setShooterVelocityRPM(double velocity) {
        shooterLeftGoalVelRPM = velocity;
        shooterRightGoalVelRPM = velocity * ScoringConstants.shooterOffsetAdjustment;
    }

    @Override
    public void setKickerVolts(double volts) {
        kickerVolts = volts;
    }

    @Override
    public void setOverrideMode(boolean override) {
        this.override = override;
    }

    @Override
    public void setOverrideVolts(double volts) {
        shooterLeftAppliedVolts = volts;
        shooterRightAppliedVolts = volts;
    }

    @Override
    public void setPID(double p, double i, double d) {
        shooterLeftController.setPID(p, i, d);
        shooterRightController.setPID(p, i, d);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        shooterLeftSim.update(SimConstants.loopTime);
        shooterRightSim.update(SimConstants.loopTime);

        inputs.shooterLeftVelocityRPM =
                shooterLeftSim.getAngularVelocityRadPerSec()
                        * ConversionConstants.kRadiansPerSecondToRPM;
        inputs.shooterLeftStatorCurrentAmps = shooterLeftSim.getCurrentDrawAmps();

        inputs.shooterRightVelocityRPM =
                shooterRightSim.getAngularVelocityRadPerSec()
                        * ConversionConstants.kRadiansPerSecondToRPM;
        inputs.shooterRightStatorCurrentAmps = shooterRightSim.getCurrentDrawAmps();

        inputs.shooterLeftAppliedVolts = shooterLeftAppliedVolts;
        inputs.shooterRightAppliedVolts = shooterRightAppliedVolts;

        inputs.kickerAppliedVolts = kickerVolts;
        inputs.kickerStatorCurrentAmps = 0.0;

        inputs.noteInShooter = false;
    }

    @Override
    public void applyOutputs(ShooterOutputs outputs) {

        outputs.shooterLeftGoalVelocityRPM = shooterLeftGoalVelRPM;
        outputs.shooterRightGoalVelocityRPM = shooterRightGoalVelRPM;

        if (!override) {
            shooterLeftAppliedVolts =
                    shooterFeedforward.calculate(shooterLeftSim.getAngularVelocityRadPerSec())
                            + shooterLeftController.calculate(
                                    shooterLeftSim.getAngularVelocityRadPerSec(),
                                    outputs.shooterLeftGoalVelocityRPM
                                            * ConversionConstants.kRPMToRadiansPerSecond);
            shooterRightAppliedVolts =
                    shooterFeedforward.calculate(shooterRightSim.getAngularVelocityRadPerSec())
                            + shooterRightController.calculate(
                                    shooterRightSim.getAngularVelocityRadPerSec(),
                                    outputs.shooterRightGoalVelocityRPM
                                            * ConversionConstants.kRPMToRadiansPerSecond);
        }

        shooterLeftSim.setInputVoltage(shooterLeftAppliedVolts);
        shooterRightSim.setInputVoltage(shooterRightAppliedVolts);
    }
}
