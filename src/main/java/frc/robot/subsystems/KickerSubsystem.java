package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.KickerConstants.*;

public class KickerSubsystem extends SubsystemBase {
    private final TalonFX motorKicker = new TalonFX(KickerMotorId, "rio");

    public KickerSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = KickerGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorKicker.getConfigurator().apply(m_motorConfig);
    }

    /// Enables the kicker to move balls from the intake to the shooter.
    public void kick() {
        motorKicker.set(1.0);
    }

    /// Disables the kicker.
    public void stop() {
        motorKicker.set(0);
    }
}