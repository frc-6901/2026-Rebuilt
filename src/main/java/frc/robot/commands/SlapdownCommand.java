package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SlapdownSubsystem;

public class SlapdownCommand extends Command {
    private final SlapdownSubsystem slapdown;

    public SlapdownCommand(SlapdownSubsystem slapdown) {
        this.slapdown = slapdown;
        addRequirements(slapdown);
    }

    @Override
    public void initialize() {
        if (slapdown.isSlapdownDeployed) {
            slapdown.retractSlapdown();
        } else {
            slapdown.slapdown();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // slapdown.stop();
    }

}