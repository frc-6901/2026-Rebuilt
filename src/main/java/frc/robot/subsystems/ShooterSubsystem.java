package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX motorLeft = new TalonFX(ShooterConstants.LeftMotorId);
    private final TalonFX motorRight = new TalonFX(ShooterConstants.RightMotorId);

    public ShooterSubsystem() {
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.Slot0 = ShooterConstants.ShooterGains;
        motorLeft.getConfigurator().apply(leftMotorConfig);

        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.Slot0 = ShooterConstants.ShooterGains;
        motorRight.getConfigurator().apply(rightMotorConfig);
    }

    // Sets the power of both motors.
    //
    // The left motor is set to the given power, while the right motor is set to the
    // negative of that power to ensure they spin in opposite directions.
    public void setPower(double power) {
        motorLeft.setControl(new DutyCycleOut(power));
        motorRight.setControl(new DutyCycleOut(-power));
    }

    // Disables both motors by setting their power to 0.
    public void stop() {
        motorLeft.setControl(new DutyCycleOut(0));
        motorRight.setControl(new DutyCycleOut(0));
    }
}
