package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

<<<<<<< HEAD
public class IntakeSubsystem extends SubsystemBase {
=======
public class IntakeSubsystem extends SubsystemBase{
>>>>>>> 23dcefcc743496772d49461f9afaef3e155959fb
    private final TalonFX motorIntake = new TalonFX(IntakeMotorId);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public IntakeSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = IntakeGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorIntake.getConfigurator().apply(m_motorConfig);
    }

<<<<<<< HEAD
    // Sets the power of intake motor.
    public void setPower(double power) {
        // rps prolly needs to be multipled with some constant
=======
    //Sets the power of intake motor.
    public void setPower(double power) {
        //rps prolly needs to be multipled with some constant
>>>>>>> 23dcefcc743496772d49461f9afaef3e155959fb
        motorIntake.setControl(m_request.withVelocity(rps));
    }

    public void stop() {
        motorIntake.setControl(m_request.withVelocity(0));

    }
}
