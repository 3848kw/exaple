package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.utils.sim.PhysicsSim;

@Logged
public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterLeader;
    private final SparkMax shooterFollower;
    
    private final RelativeEncoder leaderEncoder;
    private final SparkClosedLoopController closedLoopController;

    private double wantedFlyWheelVelocity;

    private final SysIdRoutine flywheelRoutine;
    
    public ShooterSubsystem() {
        shooterLeader = new SparkMax(ShooterConstants.leaderID, null);
        shooterFollower = new SparkMax(ShooterConstants.followMotorID, null);

        leaderEncoder = shooterLeader.getEncoder();
        closedLoopController = shooterLeader.getClosedLoopController();

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.inverted(true); // CounterClockwise_Positive equivalent
        leaderConfig.smartCurrentLimit(60);

        // Configure PID for velocity control
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.p(ShooterConstants.FlyWheelPID.P);
        closedLoopConfig.i(ShooterConstants.FlyWheelPID.I);
        closedLoopConfig.d(ShooterConstants.FlyWheelPID.D);
        // SparkMax velocity reference uses the velocity coefficient from config
        // S (static feedforward) and A (acceleration feedforward) can be set via auxiliary PID
        leaderConfig.apply(closedLoopConfig);

        shooterLeader.configure(leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.follow(shooterLeader); // Follower mode (default is opposed)
        shooterFollower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        flywheelRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::setFlywheelVoltage,
                log -> {
                    log.motor("Flywheel")
                        .voltage(Volts.of(shooterLeader.getAppliedOutput() * shooterLeader.getBusVoltage()))
                        .angularVelocity(RotationsPerSecond.of(leaderEncoder.getVelocity()))
                       .angularPosition(Units.Rotations.of(leaderEncoder.getPosition()));

                },
                this
            )
        );

        SmartDashboard.putData("SysId/Flywheel/Quasi Fwd", flywheelSysidQuadradic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysId/Flywheel/Dyn Fwd", flywheelSysidDynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysId/Flywheel/Quasi Rev", flywheelSysidQuadradic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("SysId/Flywheel/Dyn Rev", flywheelSysidDynamic(SysIdRoutine.Direction.kReverse));

    }

    private void setFlywheelVoltage(Voltage volts) {
        shooterLeader.setVoltage(volts.in(Volts));
    }
    
    public double getFlyWheelVelocity() {
        return leaderEncoder.getVelocity() * 60; // Convert rotations/sec to RPM
    }

    public double getFlyWheelWantedVelocity() {
        return wantedFlyWheelVelocity;
    }

    private void setFlyWheelVelocity(double velocity) {
        wantedFlyWheelVelocity = velocity;
        // Set velocity in rotations per second (SparkMax uses rotations for position/velocity)
        closedLoopController.setReference(velocity / 60, ControlType.kVelocity);
    }

    public void stop() {
        shooterLeader.stopMotor();
        shooterFollower.stopMotor();
        wantedFlyWheelVelocity = 0;
    }

    public Trigger ready() {
        return new Trigger(() ->
            getFlyWheelVelocity() > (wantedFlyWheelVelocity - 500)
        );
    }

    public Command holdVelocity(DoubleSupplier flywheelVelocity) {
        return Commands.runEnd(
            () -> {
                setFlyWheelVelocity(flywheelVelocity.getAsDouble());
            },
            this::stop,
            this
        ).withName("ShooterHoldVelocity");
    }

    public Command flywheelSysidQuadradic(SysIdRoutine.Direction dir) {
        return flywheelRoutine.quasistatic(dir).withTimeout(10).finallyDo(this::stop);
    }

    public Command flywheelSysidDynamic(SysIdRoutine.Direction dir) {
        return flywheelRoutine.dynamic(dir).withTimeout(10).finallyDo(this::stop);
    }
}
