package frc.robot.subsystems.scoring;

import coppercore.controls.Tunable;
import coppercore.math.InterpolateDouble;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ScoringConstants;
import frc.robot.utils.FieldFinder;
import frc.robot.utils.FieldFinder.FieldLocations;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ScoringSubsystem extends SubsystemBase implements Tunable {
    private final ShooterIO shooterIo;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
    private final ShooterIOOutputsAutoLogged shooterOutputs = new ShooterIOOutputsAutoLogged();

    private final AimerIO aimerIo;
    private final AimerIOInputsAutoLogged aimerInputs = new AimerIOInputsAutoLogged();
    private final AimerIOOutputsAutoLogged aimerOutputs = new AimerIOOutputsAutoLogged();

    private final Timer shootTimer = new Timer();

    private final Timer sourceIntakeTimer = new Timer();
    private boolean sourceTimerStarted = false;

    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
    private Supplier<Vector<N2>> velocitySupplier = () -> VecBuilder.fill(0.0, 0.0);
    private DoubleSupplier elevatorPositionSupplier = () -> 0.0;
    private Supplier<Boolean> driveAllignedSupplier = () -> true;

    private final InterpolateDouble shooterInterpolated;
    private final InterpolateDouble aimerInterpolated;
    private final InterpolateDouble aimerAngleTolerance;

    private double shooterGoalVelocityRPMTuning = 0.0;
    private double aimerGoalAngleRadTuning = 0.0;
    private double kickerVoltsTuning = 0.0;

    private boolean overrideIntake = false;
    private boolean overrideShoot = false;
    private boolean overrideStageAvoidance = false;
    private boolean overrideBeamBreak = false;

    private Mechanism2d mechanism;
    private MechanismRoot2d rootMechanism;
    private MechanismLigament2d aimMechanism;

    public enum ScoringState {
        IDLE,
        INTAKE,
        SPIT,
        SOURCE_INTAKE,
        PRIME,
        AMP_PRIME,
        SHOOT,
        AMP_SHOOT,
        ENDGAME,
        TUNING,
        OVERRIDE,
        TEMPORARY_SETPOINT
    }

    public enum ScoringAction {
        WAIT,
        INTAKE,
        SPIT,
        SOURCE_INTAKE,
        AIM,
        AMP_AIM,
        SHOOT,
        ENDGAME,
        TUNING,
        OVERRIDE,
        TEMPORARY_SETPOINT,
        TRAP_SCORE
    }

    private ScoringState state = ScoringState.IDLE;

    private ScoringAction action = ScoringAction.WAIT;

    private int temporarySetpointSlot = 0;
    private double temporarySetpointPosition = 0.0;

    private boolean readyToShoot = false;

    public ScoringSubsystem(ShooterIO shooterIo, AimerIO aimerIo) {

        this.shooterIo = shooterIo;
        this.aimerIo = aimerIo;

        shooterInterpolated =
                new InterpolateDouble(ScoringConstants.synced.getObject().getShooterMap());

        aimerInterpolated =
                new InterpolateDouble(
                        ScoringConstants.synced.getObject().getAimerMap(),
                        0.0,
                        ScoringConstants.synced.getObject().aimMaxAngleRadians);

        aimerAngleTolerance =
                new InterpolateDouble(ScoringConstants.synced.getObject().aimerToleranceTable());

        if (!Robot.isReal()) {
            mechanism = new Mechanism2d(2.2, 2.0);
            rootMechanism = mechanism.getRoot("scoring", 0.6, 0.3);
            aimMechanism = rootMechanism.append(new MechanismLigament2d("aimer", 0.5, 0.0));
        }
    }

    public void setAction(ScoringAction action) {
        this.action = action;
    }

    public void setBrakeMode(boolean brake) {
        aimerIo.setBrakeMode(brake);
    }

    public boolean atAimerGoalPosition() {
        return Math.abs(aimerInputs.aimAngleRad - aimerInputs.aimGoalAngleRad) < 0.2;
    }

    private void idle() {
        aimerIo.setAimAngleRad(-0.03, true);
        shooterIo.setShooterVelocityRPM(0);
        shooterIo.setKickerVolts(0);

        aimerIo.setOverrideMode(false);
        shooterIo.setOverrideMode(false);
        shooterIo.setOverrideVolts(0);

        Logger.recordOutput("scoring/aimGoal", 0.0);

        SmartDashboard.putBoolean("Has Note", shooterInputs.bannerSensor);
        SmartDashboard.putNumber("Aimer Location", aimerInputs.aimAngleRad);

        if ((!hasNote() || overrideIntake) && action == ScoringAction.INTAKE) {
            state = ScoringState.INTAKE;
        } else if ((!hasNote() || overrideIntake) && action == ScoringAction.SOURCE_INTAKE) {
            state = ScoringState.SOURCE_INTAKE;
            sourceTimerStarted = false;
        } else if (action == ScoringAction.SPIT) {
            state = ScoringState.SPIT;
        } else if (action == ScoringAction.AIM || action == ScoringAction.SHOOT) {
            state = ScoringState.PRIME;
            aimerIo.setAimAngleRad(aimerInputs.aimAngleRad + 0.001, true);
            shooterIo.setShooterVelocityRPM(2000);
        } else if (action == ScoringAction.AMP_AIM) {
            state = ScoringState.AMP_PRIME;
        } else if (action == ScoringAction.ENDGAME || action == ScoringAction.TRAP_SCORE) {
            state = ScoringState.ENDGAME;
        } else if (action == ScoringAction.TUNING) {
            state = ScoringState.TUNING;
            SmartDashboard.putNumber("Test-Mode/AimerGoal", aimerGoalAngleRadTuning);
            SmartDashboard.putNumber("Test-Mode/ShooterGoal", shooterGoalVelocityRPMTuning);
        } else if (action == ScoringAction.OVERRIDE) {
            state = ScoringState.OVERRIDE;
        }
    }

    private void intake() {
        if (!aimerAtIntakePosition()) {
            aimerIo.setAimAngleRad(
                    ScoringConstants.synced.getObject().intakeAngleToleranceRadians, true);
        }
        shooterIo.setKickerVolts(ScoringConstants.synced.getObject().kickerIntakeVolts);

        if ((hasNote()) || action != ScoringAction.INTAKE) {
            state = ScoringState.IDLE;
        }
    }

    private void sourceIntake() {
        aimerIo.setAimAngleRad(0.35, true);
        shooterIo.setKickerVolts(-1);

        shooterIo.setOverrideMode(true);
        shooterIo.setOverrideVolts(-2);

        if (hasNote() && !sourceTimerStarted) {
            sourceIntakeTimer.reset();
            sourceIntakeTimer.start();

            sourceTimerStarted = true;
        }

        if ((sourceIntakeTimer.get() > 0.15 && sourceTimerStarted)
                || action != ScoringAction.SOURCE_INTAKE) {
            sourceIntakeTimer.stop();

            shooterIo.setOverrideMode(false);
            sourceTimerStarted = false;
            state = ScoringState.IDLE;
        }
    }

    private void spit() {
        shooterIo.setKickerVolts(-4.0);

        if (action != ScoringAction.SPIT) {
            state = ScoringState.IDLE;
        }
    }

    private void prime() {
        double distanceToGoal = findDistanceToGoal();
        Logger.recordOutput("scoring/aimGoal", getAimerAngle(distanceToGoal));
        shooterIo.setShooterVelocityRPM(shooterInterpolated.getValue(distanceToGoal));
        aimerIo.setAimAngleRad(getAimerAngle(distanceToGoal), false);
        if (!overrideBeamBreak) {
            shooterIo.setKickerVolts(
                    hasNote() ? 0.0 : ScoringConstants.synced.getObject().kickerIntakeVolts);
        }

        boolean shooterReady =
                shooterInputs.shooterLeftVelocityRPM
                                < (shooterOutputs.shooterLeftGoalVelocityRPM
                                        + ScoringConstants.synced.getObject()
                                                .shooterUpperVelocityMarginRPM)
                        && shooterInputs.shooterLeftVelocityRPM
                                > (shooterOutputs.shooterLeftGoalVelocityRPM
                                        - ScoringConstants.synced.getObject()
                                                .shooterLowerVelocityMarginRPM);
        boolean aimReady =
                Math.abs(aimerInputs.aimAngleRad - aimerInputs.aimGoalAngleRad)
                                < aimerAngleTolerance.getValue(distanceToGoal)
                        && Math.abs(aimerInputs.aimVelocityErrorRadPerSec)
                                < ScoringConstants.synced.getObject().aimAngleVelocityMargin;
        boolean driveReady = driveAllignedSupplier.get();
        boolean fieldLocationReady = true;

        if (!DriverStation.getAlliance().isPresent()) {
            fieldLocationReady = true;
        } else {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    fieldLocationReady =
                            FieldFinder.whereAmI(poseSupplier.get()) == FieldLocations.BLUE_WING
                                    || (FieldFinder.whereAmI(poseSupplier.get())
                                                    == FieldLocations.MIDDLE
                                            && DriverStation.isTeleop());
                    break;
                case Red:
                    fieldLocationReady =
                            FieldFinder.whereAmI(poseSupplier.get()) == FieldLocations.RED_WING
                                    || (FieldFinder.whereAmI(poseSupplier.get())
                                                    == FieldLocations.MIDDLE
                                            && DriverStation.isTeleop());
                    break;
            }
        }

        boolean notePresent = overrideBeamBreak ? true : hasNote();

        boolean primeReady = shooterReady && aimReady && driveReady && fieldLocationReady;
        readyToShoot = primeReady && notePresent;

        Logger.recordOutput("scoring/shooterReady", shooterReady);
        Logger.recordOutput("scoring/aimReady", aimReady);
        Logger.recordOutput("scoring/driverReady", driveReady);
        Logger.recordOutput("scoring/fieldLocationReady", fieldLocationReady);
        Logger.recordOutput("scoring/notePresent", notePresent);
        Logger.recordOutput("scoring/primeReady", primeReady);
        Logger.recordOutput("scoring/readyToShoot", readyToShoot);

        if (action != ScoringAction.SHOOT && action != ScoringAction.AIM) {
            state = ScoringState.IDLE;
        } else if (action == ScoringAction.SHOOT && (readyToShoot || overrideShoot)) {
            state = ScoringState.SHOOT;

            shootTimer.reset();
            shootTimer.start();
        }
    }

    private void ampPrime() {
        shooterIo.setShooterVelocityRPM(ScoringConstants.synced.getObject().shooterAmpVelocityRPM);
        aimerIo.setAimAngleRad(1.65, true);
        if (action != ScoringAction.SHOOT && action != ScoringAction.AMP_AIM) {
            state = ScoringState.IDLE;
        } else if (action == ScoringAction.SHOOT) {
            state = ScoringState.AMP_SHOOT;

            shootTimer.reset();
            shootTimer.start();
        }
    }

    private void shoot() {
        double distancetoGoal = findDistanceToGoal();

        double shootRPM = shooterInterpolated.getValue(distancetoGoal);
        shooterIo.setShooterVelocityRPM(shootRPM);
        double aimAngleRad = aimerInterpolated.getValue(distancetoGoal);
        aimerIo.setAimAngleRad(aimAngleRad, false);

        shooterIo.setKickerVolts(10);

        if (shootTimer.get() > 0.5) { // TODO: Tune time
            state = ScoringState.PRIME;

            shootTimer.stop();
        }
    }

    private void ampShoot() {
        shooterIo.setKickerVolts(10);

        if (shootTimer.get() > 1.0) { // TODO: Tune time
            state = ScoringState.AMP_PRIME;

            shootTimer.stop();
        }
    }

    private void endgame() {
        aimerIo.setAimAngleRad(Math.PI / 2, true);
        shooterIo.setShooterVelocityRPM(ScoringConstants.synced.getObject().shooterAmpVelocityRPM);
        shooterIo.setKickerVolts(action == ScoringAction.TRAP_SCORE ? 10 : 0);
        if (action != ScoringAction.ENDGAME && action != ScoringAction.TRAP_SCORE) {
            state = ScoringState.IDLE;
        }
    }

    private void tuning() {
        shooterGoalVelocityRPMTuning = SmartDashboard.getNumber("Test-Mode/ShooterGoal", 0.0);
        aimerGoalAngleRadTuning = SmartDashboard.getNumber("Test-Mode/AimerGoal", 0.0);
        shooterIo.setShooterVelocityRPM(shooterGoalVelocityRPMTuning);
        aimerIo.setAimAngleRad(aimerGoalAngleRadTuning, true);
        shooterIo.setKickerVolts(kickerVoltsTuning);

        if (action != ScoringAction.TUNING) {
            state = ScoringState.IDLE;
        }
    }

    private void override() {
        shooterIo.setKickerVolts(kickerVoltsTuning);

        aimerIo.setOverrideMode(true);
        shooterIo.setOverrideMode(true);

        if (action == ScoringAction.TEMPORARY_SETPOINT) {
            state = ScoringState.TEMPORARY_SETPOINT;
        }

        if (action != ScoringAction.OVERRIDE) {
            state = ScoringState.IDLE;
        }
    }

    private void temporarySetpoint() {
        shooterIo.setKickerVolts(kickerVoltsTuning);

        aimerIo.setOverrideMode(false);
        shooterIo.setOverrideMode(false);

        if (action != ScoringAction.TEMPORARY_SETPOINT) {
            state = ScoringState.OVERRIDE;

            setVolts(0.0, temporarySetpointSlot);
        }
    }

    private double findDistanceToGoal() {
        Translation2d speakerPose = new Translation2d(); // AllianceUtil.getFieldToSpeaker();
        Pose2d robotPose = poseSupplier.get();
        double distancetoGoal =
                Math.sqrt(
                        Math.pow(Math.abs(robotPose.getX() - speakerPose.getX()), 2)
                                + Math.pow(Math.abs(robotPose.getY() - speakerPose.getY()), 2));
        return distancetoGoal;
    }

    public boolean hasNote() {
        return shooterInputs.bannerSensor;
    }

    public boolean aimerAtIntakePosition() {
        return aimerInputs.aimAngleRad
                > ScoringConstants.synced.getObject().intakeAngleToleranceRadians;
        // return true;\][]
    }

    public boolean canIntake() {
        return aimerAtIntakePosition() && !hasNote();
    }

    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public void setVelocitySupplier(Supplier<Vector<N2>> velocitySupplier) {
        this.velocitySupplier = velocitySupplier;
    }

    public void setElevatorPositionSupplier(DoubleSupplier elevatorPositionSupplier) {
        this.elevatorPositionSupplier = elevatorPositionSupplier;
    }

    public void setDriveAllignedSupplier(Supplier<Boolean> driveAllignedSupplier) {
        this.driveAllignedSupplier = driveAllignedSupplier;
    }

    public void enabledInit() {
        aimerIo.resetPID();

        setOverrideStageAvoidance(false);
        setOverrideShoot(false);
        aimerIo.setAimAngleRad(aimerInputs.aimAngleRad, true);
    }

    @Override
    public void periodic() {

        if (!SmartDashboard.containsKey("Aimer Offset")) {
            SmartDashboard.putNumber(
                    "Aimer Offset", ScoringConstants.synced.getObject().aimerStaticOffset);
        }

        if (!SmartDashboard.containsKey("Beam Break Overridden")) {
            SmartDashboard.putBoolean("Beam Break Overridden", overrideBeamBreak);
        }

        overrideBeamBreak = SmartDashboard.getBoolean("Beam Break Overridden", overrideBeamBreak);

        if (state == ScoringState.TEMPORARY_SETPOINT) {
            aimerIo.setAngleClampsRad(
                    ScoringConstants.synced.getObject().aimMinAngleRadians,
                    ScoringConstants.synced.getObject().aimMaxAngleRadians);
        } else if (state != ScoringState.TUNING
                && state != ScoringState.ENDGAME
                && state != ScoringState.IDLE
                // && Math.abs(elevatorPositionSupplier.getAsDouble()) < 0.2
                && !overrideStageAvoidance) {
            aimerIo.setAngleClampsRad(ScoringConstants.synced.getObject().aimMinAngleRadians, 0);
        }

        aimerIo.controlAimAngleRad();

        aimerIo.setOverrideMode(state == ScoringState.OVERRIDE);
        shooterIo.setOverrideMode(state == ScoringState.OVERRIDE);

        Logger.recordOutput("scoring/State", state.toString());
        Logger.recordOutput("scoring/Action", action.toString());

        Logger.recordOutput(
                "scoring/Aimer3d",
                new Pose3d(-0.255, 0.2, 0.502, new Rotation3d(0, -aimerInputs.aimAngleRad, 0)));

        Logger.recordOutput("scoring/readyToShoot", readyToShoot);
        Logger.recordOutput("scoring/overrideShoot", overrideShoot);
        Logger.recordOutput("scoring/overrideStageAvoidance", overrideStageAvoidance);

        Logger.recordOutput("scoring/distance", findDistanceToGoal());

        if (!Robot.isReal()) {
            aimMechanism.setAngle(Units.radiansToDegrees(aimerInputs.aimAngleRad));
            Logger.recordOutput("scoring/mechanism2d", mechanism);
        }

        switch (state) {
            case IDLE:
                idle();
                break;
            case INTAKE:
                intake();
                break;
            case SOURCE_INTAKE:
                sourceIntake();
                break;
            case SPIT:
                spit();
                break;
            case PRIME:
                prime();
                break;
            case AMP_PRIME:
                ampPrime();
                break;
            case SHOOT:
                shoot();
                break;
            case AMP_SHOOT:
                ampShoot();
                break;
            case ENDGAME:
                endgame(); // TODO: Later
                break;
            case TUNING:
                tuning();
                break;
            case OVERRIDE:
                override();
                break;
            case TEMPORARY_SETPOINT:
                temporarySetpoint();
                break;
        }

        // If the robot is disabled, the pid should not be winding up
        if (DriverStation.isDisabled()) {
            aimerIo.resetPID();
            aimerIo.setAimAngleRad(aimerInputs.aimAngleRad, true);
        }

        shooterIo.updateInputs(shooterInputs);
        aimerIo.updateInputs(aimerInputs);

        shooterIo.applyOutputs(shooterOutputs);
        aimerIo.applyOutputs(aimerOutputs);

        Logger.processInputs("scoring/shooter", shooterInputs);
        Logger.processInputs("scoring/aimer", aimerInputs);
    }

    public void setTuningKickerVolts(double kickerVoltsTuning) {
        this.kickerVoltsTuning = kickerVoltsTuning;
    }

    public void setOverrideIntake(boolean overrideIntake) {
        this.overrideIntake = overrideIntake;
    }

    public void setOverrideShoot(boolean overrideShoot) {
        this.overrideShoot = overrideShoot;
    }

    public void setOverrideStageAvoidance(boolean overrideStageAvoidance) {
        this.overrideStageAvoidance = overrideStageAvoidance;
    }

    public void setOverrideBeamBrake(boolean overrideBeamBrake) {
        this.overrideBeamBreak = overrideBeamBrake;
    }

    public void setArmDisabled(boolean disabled) {
        aimerIo.setMotorDisabled(disabled);
    }

    public double getAimerAngle(double distance) {
        return aimerInterpolated.getValue(distance)
                + SmartDashboard.getNumber(
                        "Aimer Offset", ScoringConstants.synced.getObject().aimerStaticOffset);
    }

    @Override
    public double getPosition(int slot) {
        switch (slot) {
                // Aimer
            case 0:
                return aimerInputs.aimAngleRad;
                // Shooter
            case 1:
                return shooterInputs.shooterLeftVelocityRPM;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public double getVelocity(int slot) {
        switch (slot) {
                // Aimer
            case 0:
                return aimerInputs.aimVelocityRadPerSec;
                // Shooter
            case 1:
                return shooterInputs.shooterLeftVelocityRPM;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public double getConversionFactor(int slot) {
        switch (slot) {
                // Aimer
            case 0:
                return 1.0;
                // Shooter
            case 1:
                return 1.0;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public void setVolts(double volts, int slot) {
        switch (slot) {
                // Aimer
            case 0:
                aimerIo.setOverrideVolts(volts);
                break;
                // Shooter
            case 1:
                shooterIo.setOverrideVolts(volts);
                break;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public void setPID(double p, double i, double d, int slot) {
        switch (slot) {
                // Aimer
            case 0:
                aimerIo.setPID(p, i, d);
                break;
                // Shooter
            case 1:
                shooterIo.setPID(p, i, d);
                break;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public void setMaxProfileProperties(double maxVelocity, double maxAcceleration, int slot) {
        switch (slot) {
                // Aimer
            case 0:
                aimerIo.setMaxProfile(maxVelocity, maxAcceleration);
                break;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public void setFF(double kS, double kV, double kA, double kG, int slot) {
        switch (slot) {
                // Aimer
            case 0:
                aimerIo.setFF(kS, kV, kA, kG);
                break;
                // Shooter
            case 2:
                shooterIo.setFF(kS, kV, kA);
                break;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public void runToPosition(double position, int slot) {
        state = ScoringState.TEMPORARY_SETPOINT;

        temporarySetpointPosition = position * getConversionFactor(slot);
        temporarySetpointSlot = slot;

        switch (slot) {
                // Aimer
            case 0:
                aimerIo.setAimAngleRad(temporarySetpointPosition, true);
                break;
                // Shooter
            case 1:
                shooterIo.setShooterVelocityRPM(temporarySetpointPosition);
                break;
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    public ScoringAction getCurrentAction() {
        return action;
    }

    public ScoringState getCurrentState() {
        return state;
    }

    public boolean readyToShoot() {
        return readyToShoot;
    }

    public void setAimerStatorCurrentLimit(double limit) {
        aimerIo.setStatorCurrentLimit(limit);
    }
}
