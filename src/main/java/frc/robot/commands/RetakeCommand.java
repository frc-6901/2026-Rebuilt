package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RetakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RetakeCommand extends Command {
    private IntakeSubsystem intake;
    private Timer timer;

    public RetakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        this.timer = new Timer();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.restart();
        intake.outtake(RetakeConstants.OuttakeRPS);
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(RetakeConstants.OuttakeDuration)) {
            intake.intake(RetakeConstants.IntakeRPS);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(RetakeConstants.TotalDuration);
    }
}
