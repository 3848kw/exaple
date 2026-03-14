package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.utils.sim.PhysicsSim;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;

@Logged
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax deployMotor;
    private final SparkMax intakeMotor;

    private final RelativeEncoder deployEncoder;
    private final SparkClosedLoopController deployController;

    private IntakeSimulation intakeSim;
    private SwerveDriveSimulation swerveDriveSim;
    
    private Optional<TurretSubsystem> turret = Optional.empty();
    private Optional<HoodSubsystem> hood = Optional.empty();

    public IntakeSubsystem(Optional<SwerveDriveSimulation> swerveDriveSim) {
        deployMotor = new SparkMax(IntakeConstants.angleMotorId, null);
        intakeMotor = new SparkMax(IntakeConstants.intakeMotorId, null);

        deployEncoder = deployMotor.getEncoder();
        deployController = deployMotor.getClosedLoopController();

        SparkMaxConfig deployConfig = new SparkMaxConfig();
        deployConfig.idleMode(IdleMode.kBrake);
        deployConfig.inverted(false);
        deployConfig.smartCurrentLimit(60);

        // Configure closed loop for position control (MotionMagic equivalent)
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.p(IntakeConstants.p);
        closedLoopConfig.i(IntakeConstants.i);
        closedLoopConfig.d(IntakeConstants.d);
        // SparkMax doesn't have direct gravity feedforward; can add via auxiliary PID if needed
        deployConfig.apply(closedLoopConfig);
        // Note: SparkMax's position control is essentially PID with optional motion profiling
        // For MotionMagic behavior, consider using SmartMotion in config if needed

        deployMotor.configure(deployConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.inverted(false);
        intakeConfig.smartCurrentLimit(50);
        intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        double rioUptime = Timer.getFPGATimestamp();
        if (rioUptime < 180) {
            deployEncoder.setPosition(IntakeConstants.stowPosition);
        }

        if(Robot.isSimulation()) {
            PhysicsSim.getInstance().addSparkMax(deployMotor, DCMotor.getNEO(1));
            PhysicsSim.getInstance().addSparkMax(intakeMotor, DCMotor.getNEO(1));
            this.swerveDriveSim = swerveDriveSim.get();

            intakeSim = IntakeSimulation.OverTheBumperIntake("Fuel", this.swerveDriveSim, Meters.of(0.6858), Meters.of(0.254), IntakeSide.FRONT, 50);
        }
    }
    
    public double getPosition() {
        return deployEncoder.getPosition();
    }

    public void deploy() {
        deployController.setReference(IntakeConstants.deployedPosition, ControlType.kPosition);
    }

    private void stow() {
        deployController.setReference(IntakeConstants.stowPosition, ControlType.kPosition);
    }

    private void runIntake(double duty) {
        intakeMotor.set(duty);
        if (intakeSim != null) intakeSim.startIntake();
    }

    private void stopIntake() {
        intakeMotor.set(0);
        if (intakeSim != null) intakeSim.stopIntake();
    }

    public IntakeSimulation getIntakeSim() {
        return intakeSim;
    }
    
    public void setTurretSubsystem(TurretSubsystem turretSubsystem) {
        this.turret = Optional.of(turretSubsystem);
    }

    public void setHoodSubsystem(HoodSubsystem hoodSubsystem) {
        this.hood = Optional.of(hoodSubsystem);
    }
    
    public boolean isDeployed() {
        return MathUtil.isNear(IntakeConstants.deployedPosition, getPosition(), 4);
    }
        
    public Command resetEncoderToStow() {
        return Commands.runOnce(() -> deployEncoder.setPosition(IntakeConstants.stowPosition)).ignoringDisable(true).withName("ResetIntakeToStow");
    }

    public Command resetEncoderToDeploy() {
        return Commands.runOnce(() -> deployEncoder.setPosition(IntakeConstants.deployedPosition)).ignoringDisable(true).withName("ResetIntakeToStow");
    }

    public Command deployIntake() {
        return Commands.runOnce(this::deploy, this).withName("DeployIntake");
    }
    
    public Command stowIntake() {
        Command homeTurret = turret
            .map(t -> (Command) Commands.run(() -> t.setRobotRelativeAngleDeg(0.0), t).until(t::isAtHome))
            .orElse(Commands.none());

        Command lowerHood = hood
            .map(h -> (Command) Commands.run(() -> h.setAngle(new Rotation2d()), h).until(h::isAtHome))
            .orElse(Commands.none());

        return Commands.sequence(
            Commands.runOnce(this::stopIntake, this),
            Commands.parallel(homeTurret, lowerHood),
            Commands.runOnce(this::stow, this)
        ).withName("Stow Intake");
    }

    public Command runIntakeRollers() {
        return Commands.startEnd(
            () -> {
                if (!isDeployed()) {
                    deploy();
                }
                runIntake(0.65);
            },
            () -> {
                stopIntake();
            },
            this
        ).withName("RunIntakeRollers");
    }

    public Command shootingPosition() {
        return Commands.startEnd(
            () -> {
                deployController.setReference(IntakeConstants.shootingPosition, ControlType.kPosition);
                runIntake(0.5);
            }, 
            () -> {
                deploy();
                stopIntake();
            }, 
            this
        ).withName("Intake Shooting POS");
    }
}