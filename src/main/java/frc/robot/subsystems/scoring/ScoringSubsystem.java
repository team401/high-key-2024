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
import frc.robot.constants.ModeConstants;
import frc.robot.constants.ModeConstants.Mode;
import frc.robot.constants.ScoringConstants;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.FieldFinder;
import frc.robot.utils.FieldFinder.FieldLocations;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ScoringSubsystem extends SubsystemBase implements Tunable {
    private final ShooterIO shooterIo;
    private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();
    private final ShooterOutputsAutoLogged shooterOutputs = new ShooterOutputsAutoLogged();

    private final AimerIO aimerIo;
    private final AimerInputsAutoLogged aimerInputs = new AimerInputsAutoLogged();
    private final AimerOutputsAutoLogged aimerOutputs = new AimerOutputsAutoLogged();

    private final Timer shootTimer = new Timer();

    private final Timer sourceIntakeTimer = new Timer();
    private boolean sourceTimerStarted = false;

    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
    private Supplier<Vector<N2>> velocitySupplier = () -> VecBuilder.fill(0.0, 0.0);
    private DoubleSupplier elevatorPositionSupplier = () -> 0.0;
    private Supplier<Boolean> driveAlignedSupplier = () -> true;

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
    private boolean lockNegativeAtHome = false;

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
        TEMPORARY_SETPOINT,
        PASS_PRIME,
        PASS
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
        TRAP_SCORE,
        PASS
    }

    private ScoringState state = ScoringState.IDLE;

    private ScoringAction action = ScoringAction.WAIT;

    private int temporarySetpointSlot = 0;
    private double temporarySetpointPosition = 0.0;

    private boolean readyToShoot = false;

    public ScoringSubsystem(ShooterIO shooterIo, AimerIO aimerIo) {

        this.shooterIo = shooterIo;
        this.aimerIo = aimerIo;

        shooterInterpolated = new InterpolateDouble(ScoringConstants.getShooterMap());

        aimerInterpolated =
                new InterpolateDouble(
                        ScoringConstants.getAimerMap(),
                        ScoringConstants.aimMinAngleRotations,
                        ScoringConstants.aimMaxAngleRotations);

        aimerAngleTolerance = new InterpolateDouble(ScoringConstants.aimerToleranceTable());

        if (ModeConstants.currentMode == Mode.SIM) {
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
        return Math.abs(aimerInputs.aimAngleRot - aimerInputs.aimGoalAngleRot) < 0.2;
    }

    private void idle() {
        // aimerIo.setAimAngleRad(ScoringConstants.aimMinAngleRotations + 0.001);
        // Todo: if we can trust aimer, figure out if we really need to add 0.001 to the min angle
        aimerIo.setAimAngleRot(ScoringConstants.aimMinAngleRotations);
        shooterIo.setShooterVelocityRPM(0);
        shooterIo.setKickerVolts(0);

        aimerIo.setOverrideMode(false);
        shooterIo.setOverrideMode(false);
        shooterIo.setOverrideVolts(0);

        Logger.recordOutput("scoring/aimGoal", 0.0);

        SmartDashboard.putBoolean("Has Note", shooterInputs.noteInShooter);
        SmartDashboard.putNumber("Aimer Location", aimerInputs.aimAngleRot);

        if ((!hasNote() || overrideIntake) && action == ScoringAction.INTAKE) {
            state = ScoringState.INTAKE;
        } else if ((!hasNote() || overrideIntake) && action == ScoringAction.SOURCE_INTAKE) {
            state = ScoringState.SOURCE_INTAKE;
            sourceTimerStarted = false;
        } else if (action == ScoringAction.SPIT) {
            state = ScoringState.SPIT;
        } else if (action == ScoringAction.AIM || action == ScoringAction.SHOOT) {
            state = ScoringState.PRIME;
            // aimerIo.setAimAngleRad(aimerInputs.aimAngleRad + 0.001);
            shooterIo.setShooterVelocityRPM(2000);
        } else if (action == ScoringAction.PASS) {
            state = ScoringState.PASS_PRIME;
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
        aimerIo.setAimAngleRot(ScoringConstants.aimMinAngleRotations);
        aimerIo.setNegativeHomeLockMode(true);

        if (!hasNote()) {
            shooterIo.setKickerVolts(ScoringConstants.kickerIntakeVolts);
        } else {
            shooterIo.setKickerVolts(0.0);
        }

        if ((hasNote()) || action != ScoringAction.INTAKE) {
            state = ScoringState.IDLE;
        }
    }

    private void sourceIntake() {
        aimerIo.setAimAngleRot(0.35);
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
        aimerIo.setAimAngleRot(getAimerAngle(distanceToGoal));
        if (!overrideBeamBreak) {
            shooterIo.setKickerVolts(hasNote() ? 0.0 : ScoringConstants.kickerIntakeVolts);
        }

        boolean shooterReady =
                shooterInputs.shooterLeftVelocityRPM
                                < (shooterOutputs.shooterLeftGoalVelocityRPM
                                        + ScoringConstants.shooterUpperVelocityMarginRPM)
                        && shooterInputs.shooterLeftVelocityRPM
                                > (shooterOutputs.shooterLeftGoalVelocityRPM
                                        - ScoringConstants.shooterLowerVelocityMarginRPM);
        boolean aimReady =
                Math.abs(aimerInputs.aimAngleRot - aimerInputs.aimGoalAngleRot)
                                < aimerAngleTolerance.getValue(distanceToGoal)
                        && Math.abs(aimerInputs.aimVelocityErrorRotPerSec)
                                < ScoringConstants.aimAngleVelocityMargin;
        boolean driveReady =
                (driveAlignedSupplier.get()
                        || distanceToGoal < ScoringConstants.minDistanceAlignmentNeeded);
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

        boolean primeReady = shooterReady && aimReady && driveReady /*&& fieldLocationReady*/;
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
        // shooterIo.setShooterVelocityRPM(ScoringConstants.shooterAmpVelocityRPM);
        // TODO: Test this out
        aimerIo.setAimAngleRot(ScoringConstants.ampAimerAngleRotations);
        if (action != ScoringAction.SHOOT && action != ScoringAction.AMP_AIM) {
            state = ScoringState.IDLE;
        } else if (action == ScoringAction.SHOOT) {
            state = ScoringState.AMP_SHOOT;

            shootTimer.reset();
            shootTimer.start();
        }
    }

    private void passPrime() {
        aimerIo.setAimAngleRot(ScoringConstants.passLocationRot);
        shooterIo.setShooterVelocityRPM(ScoringConstants.passShooterRpm);

        boolean notePresent = overrideBeamBreak ? true : hasNote();

        boolean shooterReady =
                shooterInputs.shooterLeftVelocityRPM
                                < (shooterOutputs.shooterLeftGoalVelocityRPM
                                        + ScoringConstants.shooterUpperVelocityMarginRPM)
                        && shooterInputs.shooterLeftVelocityRPM
                                > (shooterOutputs.shooterLeftGoalVelocityRPM
                                        - ScoringConstants.shooterLowerVelocityMarginRPM);
        boolean aimReady =
                Math.abs(aimerInputs.aimAngleRot - ScoringConstants.passLocationRot)
                                < ScoringConstants.passAngleToleranceRot
                        && Math.abs(aimerInputs.aimVelocityErrorRotPerSec)
                                < ScoringConstants.aimAngleVelocityMargin;

        boolean driveReady = driveAlignedSupplier.get();

        boolean primeReady = shooterReady && aimReady && driveReady /*&& fieldLocationReady*/;
        readyToShoot = primeReady && notePresent;

        if (action == ScoringAction.PASS && (readyToShoot || overrideShoot)) {
            state = ScoringState.PASS;

            shootTimer.reset();
            shootTimer.start();
        } else if (action == ScoringAction.WAIT) {
            state = ScoringState.IDLE;
        }

        Logger.recordOutput("scoring/shooterReady", shooterReady);
        Logger.recordOutput("scoring/aimReady", aimReady);
        Logger.recordOutput("scoring/driverReady", driveReady);
        Logger.recordOutput("scoring/notePresent", notePresent);
        Logger.recordOutput("scoring/primeReady", primeReady);
        Logger.recordOutput("scoring/readyToShoot", readyToShoot);
    }

    private void shoot() {
        double distancetoGoal = findDistanceToGoal();

        double shootRPM = shooterInterpolated.getValue(distancetoGoal);
        shooterIo.setShooterVelocityRPM(shootRPM);
        double aimAngleRad = aimerInterpolated.getValue(distancetoGoal);
        aimerIo.setAimAngleRot(aimAngleRad);

        shooterIo.setKickerVolts(12);

        if (shootTimer.get() > 0.5) { // TODO: Tune time
            state = ScoringState.PRIME;

            shootTimer.stop();
        }
    }

    private void ampShoot() {
        shooterIo.setKickerVolts(-12); // TODO: Test if this kicks note forward or backward
        aimerIo.setAimAngleRot(ScoringConstants.ampAimerAngleRotations);

        // Revert to amp prime after we shoot the note
        // This will keep the arm up and allow the driver to drive away from the amp before the arm
        // comes down
        if (action == ScoringAction.WAIT || (!hasNote() && shootTimer.get() > 1.0)) {
            state = ScoringState.AMP_PRIME;
        }

        // I see no reason why amp should not shoot within 1 second but if it takes longer than
        // that, we should probably let it

        // Therefore this is all commented out for now
        /* if (shootTimer.get() > 1.0) { // TODO: Tune time
           state = ScoringState.AMP_PRIME;

            shootTimer.stop();
        } */
    }

    private void passShoot() {
        aimerIo.setAimAngleRot(ScoringConstants.passLocationRot);
        shooterIo.setShooterVelocityRPM(ScoringConstants.passShooterRpm);

        shooterIo.setKickerVolts(12);

        if (shootTimer.get() > 0.5) { // TODO: Tune time
            state = ScoringState.PASS_PRIME;

            shootTimer.stop();
        }
    }

    private void endgame() {
        aimerIo.setAimAngleRot(Math.PI / 2);
        // shooterIo.setShooterVelocityRPM(ScoringConstants.shooterAmpVelocityRPM);
        shooterIo.setKickerVolts(action == ScoringAction.TRAP_SCORE ? 10 : 0);
        if (action != ScoringAction.ENDGAME && action != ScoringAction.TRAP_SCORE) {
            state = ScoringState.IDLE;
        }
    }

    private void tuning() {
        shooterGoalVelocityRPMTuning = SmartDashboard.getNumber("Test-Mode/ShooterGoal", 0.0);
        aimerGoalAngleRadTuning = SmartDashboard.getNumber("Test-Mode/AimerGoal", 0.0);
        shooterIo.setShooterVelocityRPM(shooterGoalVelocityRPMTuning);
        aimerIo.setAimAngleRot(aimerGoalAngleRadTuning);
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
        Translation2d speakerPose = AllianceUtil.getFieldToSpeaker();
        Pose2d robotPose = poseSupplier.get();
        double distancetoGoal =
                Math.sqrt(
                        Math.pow(Math.abs(robotPose.getX() - speakerPose.getX()), 2)
                                + Math.pow(Math.abs(robotPose.getY() - speakerPose.getY()), 2));
        return distancetoGoal;
    }

    public boolean hasNote() {
        return shooterInputs.noteInShooter;
    }

    public boolean aimerAtIntakePosition() {
        return aimerInputs.aimAngleRot
                < ScoringConstants.aimMinAngleRotations
                        + ScoringConstants.intakeAngleToleranceRotations;
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

    public void setDriveAlignedSupplier(Supplier<Boolean> driveAllignedSupplier) {
        this.driveAlignedSupplier = driveAllignedSupplier;
    }

    public void enabledInit() {
        aimerIo.resetPID();

        setOverrideStageAvoidance(false);
        setOverrideShoot(false);
        aimerIo.setAimAngleRot(aimerInputs.aimAngleRot);
    }

    @Override
    public void periodic() {
        aimerIo.setNegativeHomeLockMode(false); // This should be false in all states but intake
        // Intake will set it to true later in the loop.

        if (!SmartDashboard.containsKey("Aimer Offset")) {
            SmartDashboard.putNumber("Aimer Offset", ScoringConstants.aimerStaticOffset);
        }

        if (!SmartDashboard.containsKey("Beam Break Overridden")) {
            SmartDashboard.putBoolean("Beam Break Overridden", overrideBeamBreak);
        }

        overrideBeamBreak = SmartDashboard.getBoolean("Beam Break Overridden", overrideBeamBreak);

        if (state == ScoringState.TEMPORARY_SETPOINT) {
            aimerIo.setAngleClampsRot(
                    ScoringConstants.aimMinAngleRotations, ScoringConstants.aimMaxAngleRotations);
        } else if (state != ScoringState.TUNING
                && state != ScoringState.ENDGAME
                && state != ScoringState.IDLE
                // && Math.abs(elevatorPositionSupplier.getAsDouble()) < 0.2
                && !overrideStageAvoidance) {
            aimerIo.setAngleClampsRot(ScoringConstants.aimMinAngleRotations, 0);
        }

        aimerIo.controlAimAngleRot();

        aimerIo.setOverrideMode(state == ScoringState.OVERRIDE);
        shooterIo.setOverrideMode(state == ScoringState.OVERRIDE);

        Logger.recordOutput("scoring/State", state.toString());
        Logger.recordOutput("scoring/Action", action.toString());

        /*Logger.recordOutput(
        "scoring/Aimer3d",
        new Pose3d(
                0.1,
                0,
                0.502,
                new Rotation3d(
                        -aimerInputs.aimGoalAngleRot * 2 * Math.PI,
                        0,
                        Math.PI / 2)));*/
        Logger.recordOutput(
                "scoring/Aimer3d",
                new Pose3d(
                        -0.03,
                        0,
                        0.58,
                        new Rotation3d(
                                Math.PI / 2,
                                -Math.PI / 2 - (aimerInputs.aimGoalAngleRot) * 2 * Math.PI,
                                0)));

        Logger.recordOutput("scoring/readyToShoot", readyToShoot);
        Logger.recordOutput("scoring/overrideShoot", overrideShoot);
        Logger.recordOutput("scoring/overrideStageAvoidance", overrideStageAvoidance);

        Logger.recordOutput("scoring/distance", findDistanceToGoal());

        if (ModeConstants.currentMode == Mode.SIM) {
            aimMechanism.setAngle(Units.radiansToDegrees(aimerInputs.aimAngleRot));
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
            case PASS_PRIME:
                passPrime();
                break;
            case PASS:
                passShoot();
                break;
            case TEMPORARY_SETPOINT:
                temporarySetpoint();
                break;
        }

        // If the robot is disabled, the pid should not be winding up
        if (DriverStation.isDisabled()) {
            aimerIo.resetPID();
            aimerIo.setAimAngleRot(aimerInputs.aimAngleRot);
        }

        shooterIo.updateInputs(shooterInputs);
        aimerIo.updateInputs(aimerInputs);

        shooterIo.applyOutputs(shooterOutputs);
        aimerIo.applyOutputs(aimerOutputs);

        Logger.processInputs("scoring/shooterInputs", shooterInputs);
        Logger.processInputs("scoring/shooterOutputs", shooterOutputs);
        Logger.processInputs("scoring/aimerInputs", aimerInputs);
        Logger.processInputs("scoring/aimerOutputs", aimerOutputs);
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
                + SmartDashboard.getNumber("Aimer Offset", ScoringConstants.aimerStaticOffset);
    }

    @Override
    public double getPosition(int slot) {
        switch (slot) {
                // Aimer
            case 0:
                return aimerInputs.aimAngleRot;
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
                return aimerInputs.aimVelocityRotPerSec;
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
            case 1:
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
                aimerIo.setAimAngleRot(temporarySetpointPosition);
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

    public void setOverrideKickerVoltsDirectly(double volts) {
        /* Immediately sets kicker voltage, fully ignoring scoring state */
        shooterIo.setKickerVolts(volts);
    }

    public ShooterIO getShooterIO() {
        return shooterIo;
    }

    public AimerIO getAimerIO() {
        return aimerIo;
    }
}
