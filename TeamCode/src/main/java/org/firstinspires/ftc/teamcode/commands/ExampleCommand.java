package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcontroller.external.ftclibexamples.CommandSample.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExampleSubsystem;

public class ExampleCommand extends CommandBase {

    private final ExampleSubsystem m_subsystem = ExampleSubsystem.getInstance();


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public ExampleCommand(ExampleSubsystem subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    //Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
