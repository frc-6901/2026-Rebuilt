package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Shoots at a preset RPM configured at command creation time.
 * 
 * <p>
 * This command executes a shooting sequence using a predefined shooter speed
 * (RPM). It coordinates the shooter, kicker, and intake subsystems to perform
 * a complete shot without needing distance calculations.
 * 
 * <p>
 * Requires: {@link ShooterSubsystem}, {@link KickerSubsystem},
 * {@link IntakeSubsystem}
 */
public class PresetShootCommand extends Command {
    private final ShooterSubsystem shooter;
    private final KickerSubsystem kicker;
    private final IntakeSubsystem intake;
    private final AngularVelocity shotrps;

    /**
     * Constructs a PresetShootCommand with a specific shot RPM.
     *
     * @param shooter the shooter subsystem
     * @param kicker  the kicker subsystem
     * @param intake  the intake subsystem
     * @param shotrps the preset angular velocity (RPM) for the shot
     */
    public PresetShootCommand(ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake,
            AngularVelocity shotrps) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.intake = intake;
        this.shotrps = shotrps;

        addRequirements(shooter, kicker, intake);
    }

    @Override
    public void execute() {
        // shooter.shoot(shotrps);
        // intake.intake(IndexRPS);
        kicker.kick();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
