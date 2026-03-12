package frc.robot.utils.motors;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

public class TalonFxUtils {
    public static void configSoftLimits(TalonFX talonFX, double forwardSoftLimit, double reverseSoftLimit, boolean enabled) {
        TalonFXConfigurator config = talonFX.getConfigurator();
        SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();
        softLimitConfigs.ForwardSoftLimitEnable = enabled;
        softLimitConfigs.ReverseSoftLimitEnable = enabled;
        softLimitConfigs.ForwardSoftLimitThreshold = forwardSoftLimit;
        softLimitConfigs.ReverseSoftLimitThreshold = reverseSoftLimit;
        config.apply(softLimitConfigs);
    }

    /**
     * links the position of the CANCoder to the encoder position of the TalonFX
     * 
     * @param talonFX
     * @param cancoder
     */
    public static void configRemoteCancoder(TalonFX talonFX, CANcoder cancoder) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        talonFX.getConfigurator().apply(config);
    }

    /**
     * Factory Reset and apply default configs
     * @param talonFX
     */
    public static void applyDefaultConfigs(TalonFX talonFX) {
        TalonFXConfigurator config = talonFX.getConfigurator();
        config.apply(new TalonFXConfiguration());

        AudioConfigs audioConfigs = new AudioConfigs();
        audioConfigs.AllowMusicDurDisable = true;
        audioConfigs.BeepOnBoot = true;
        audioConfigs.BeepOnConfig = true;

        config.apply(audioConfigs);
    }
}
