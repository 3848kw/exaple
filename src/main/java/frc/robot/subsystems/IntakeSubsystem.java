package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    private final TalonFX intakeMotor;
    private final TalonFX deployMotor;

    private final DutyCycleOut intakeControl = new DutyCycleOut(0);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

    private IntakeSimulation intakeSim;
    private SwerveDriveSimulation swerveDriveSim;
    
    private Optional<TurretSubsystem> turret = Optional.empty();
    private Optional<HoodSubsystem> hood = Optional.empty();

    public IntakeSubsystem(Optional<SwerveDriveSimulation> swerveDriveSim) {
        deployMotor = new TalonFX(IntakeConstants.angleMotorId);
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorId);

        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Slot0.kP = IntakeConstants.p;
        configs.Slot0.kI = IntakeConstants.i;
        configs.Slot0.kD = IntakeConstants.d;
        configs.Slot0.kG = IntakeConstants.g;

        configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        configs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.motionMagicCriuseVelocity;
        configs.MotionMagic.MotionMagicAcceleration = IntakeConstants.motionMagicCriuseAcceleration;
        configs.MotionMagic.MotionMagicJerk = IntakeConstants.motionMagicJerk;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        deployMotor.getConfigurator().apply(configs);
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.intakeMotorStatorLimit;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeMotor.getConfigurator().apply(intakeConfig);

        double rioUptime = Timer.getFPGATimestamp();
        if (rioUptime < 180) {
            deployMotor.setPosition(IntakeConstants.stowPosition);
        }

        if(Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalon(intakeMotor, DCMotor.getKrakenX44(1));
            PhysicsSim.getInstance().addTalon(deployMotor, DCMotor.getKrakenX44(1));
            deployMotor.getConfigurator().apply(new Slot0Configs().withKP(0.5));
            this.swerveDriveSim = swerveDriveSim.get();

            intakeSim = IntakeSimulation.OverTheBumperIntake("Fuel", this.swerveDriveSim, Meters.of(0.6858), Meters.of(0.254), IntakeSide.FRONT, 50);
        }
    }
    
    public double getPosition() {
        return deployMotor.getPosition().getValueAsDouble();
    }

    public void deploy() {
        deployMotor.setControl(positionControl.withPosition(IntakeConstants.deployedPosition));
    }

    private void stow() {
        deployMotor.setControl(positionControl.withPosition(IntakeConstants.stowPosition));
    }

    private void runIntake(double duty) {
        intakeMotor.setControl(intakeControl.withOutput(duty));
        if (intakeSim != null) intakeSim.startIntake();
    }

    private void stopIntake() {
        intakeMotor.setControl(intakeControl.withOutput(0));
        if (intakeSim != null) intakeSim.stopIntake();
    }

    public IntakeSimulation getIntakeSim() {
        return intakeSim;
    }
    
    /**
     * Sets the turret subsystem reference for coordination.
     * When intake is stowed, turret will be commanded to home position.
     */
    public void setTurretSubsystem(TurretSubsystem turretSubsystem) {
        this.turret = Optional.of(turretSubsystem);
    }

    /**
     * Sets the hood subsystem reference for coordination.
     * When intake is stowed, hood will be commanded to home position.
     */
    public void setHoodSubsystem(HoodSubsystem hoodSubsystem) {
        this.hood = Optional.of(hoodSubsystem);
    }
    
    /**
     * Checks if intake is deployed.
     */
    public boolean isDeployed() {
        return MathUtil.isNear(IntakeConstants.deployedPosition, getPosition(), 4);
    }
        
    /**
     * Resets the deploy motor encoder so the current physical position
     * is treated as the stow position. Use this from the dashboard when
     * the robot is known to be at stow and the encoder has drifted.
     */
    public Command resetEncoderToStow() {
        return Commands.runOnce(() -> deployMotor.setPosition(IntakeConstants.stowPosition)).ignoringDisable(true).withName("ResetIntakeToStow");
    }

    public Command resetEncoderToDeploy() {
        return Commands.runOnce(() -> deployMotor.setPosition(IntakeConstants.deployedPosition)).ignoringDisable(true).withName("ResetIntakeToStow");
    }

    /**
     * Deploys the intake mechanism (angle motor only).
     * Does NOT run the intake wheels.
     * Safe to use independently.
     */
    public Command deployIntake() {
        return Commands.runOnce(this::deploy, this).withName("DeployIntake");
    }
    
    /**
     * Stows the intake mechanism back to home position.
     * First ensures turret is at home and hood is lowered to prevent collision.
     * Stops wheels if they were running.
     */
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

    
    /**
     * Runs the intake rollers at the specified duty cycle.
     * Automatically deploys intake if not already deployed.
     * Continues running until the command is interrupted/cancelled.
     * Stops rollers when command ends.
     */
    public Command runIntakeRollers() {
        return Commands.startEnd(
            () -> {
                // Auto-deploy when rollers start running
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
                deployMotor.setControl(positionControl.withPosition(IntakeConstants.shootingPosition));
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