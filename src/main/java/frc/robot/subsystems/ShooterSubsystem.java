package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.utils.motors.TalonFxUtils;
import frc.robot.utils.sim.PhysicsSim;

@Logged
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterLeader;
    private final TalonFX shooterFollower ;
    private final VelocityVoltage velocityVoltage;
    private final VoltageOut voltageOut;
    private final Follower follower;
    
    private double wantedFlyWheelVelocity;

    private final SysIdRoutine flywheelRoutine;
    
    public ShooterSubsystem() {
        shooterLeader = new TalonFX(ShooterConstants.leaderID);
        shooterFollower = new TalonFX(ShooterConstants.followMotorID);

        velocityVoltage = new VelocityVoltage(0);
        voltageOut = new VoltageOut(0);

        follower = new Follower(ShooterConstants.leaderID, MotorAlignmentValue.Opposed);
        
        TalonFxUtils.applyDefaultConfigs(shooterLeader);
        TalonFxUtils.applyDefaultConfigs(shooterFollower);
        
        TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration();
        flywheelConfigs.Slot0.kP = ShooterConstants.FlyWheelPID.P;
        flywheelConfigs.Slot0.kI = ShooterConstants.FlyWheelPID.I;
        flywheelConfigs.Slot0.kD = ShooterConstants.FlyWheelPID.D;
        flywheelConfigs.Slot0.kV = ShooterConstants.FlyWheelPID.V;
        flywheelConfigs.Slot0.kS = ShooterConstants.FlyWheelPID.S;
        flywheelConfigs.Slot0.kA = ShooterConstants.FlyWheelPID.A;
        flywheelConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterLeader.getConfigurator().apply(flywheelConfigs);
        shooterLeader.setNeutralMode(NeutralModeValue.Coast);
        shooterFollower.setNeutralMode(NeutralModeValue.Coast);

        flywheelRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::setFlywheelVoltage,
                log -> {
                    log.motor("Flywheel")
                        .voltage(Volts.of(shooterLeader.getMotorVoltage().getValueAsDouble()))
                        .angularVelocity(RotationsPerSecond.of(shooterLeader.getVelocity().getValueAsDouble()))
                        .angularPosition(shooterLeader.getPosition().getValue());
                },
                this
            )
        );

        SmartDashboard.putData("SysId/Flywheel/Quasi Fwd", flywheelSysidQuadradic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysId/Flywheel/Dyn Fwd", flywheelSysidDynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysId/Flywheel/Quasi Rev", flywheelSysidQuadradic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("SysId/Flywheel/Dyn Rev", flywheelSysidDynamic(SysIdRoutine.Direction.kReverse));

        if(Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalon(shooterLeader, DCMotor.getKrakenX60(1));
            PhysicsSim.getInstance().addTalon(shooterFollower, DCMotor.getKrakenX60(1));
            shooterLeader.getConfigurator().apply(new Slot0Configs().withKV(0.12).withKP(1));
        }
    }

    private void setFlywheelVoltage(Voltage volts) {
        shooterLeader.setControl(voltageOut.withOutput(volts.in(Volts)));
        shooterFollower.setControl(follower);
    }
    
    //FlyWheel commands getters and setters
    public double getFlyWheelVelocity() {
        return shooterLeader.getVelocity().getValueAsDouble() * 60;
    }

    public double getFlyWheelWantedVelocity() {
        return wantedFlyWheelVelocity;
    }


    private void setFlyWheelVelocity(double velocity) {
        wantedFlyWheelVelocity = velocity;
        shooterLeader.setControl(velocityVoltage.withVelocity(velocity / 60));
        shooterFollower.setControl(follower);
    }

    //STOP >:c
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