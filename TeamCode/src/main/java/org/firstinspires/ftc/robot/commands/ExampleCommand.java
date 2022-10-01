package org.firstinspires.ftc.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

public class ExampleCommand extends CommandBase {

    private final ExampleSubsystem m_subsystem;
    private final DoubleSupplier m_powerSupplier;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param power the value between -1 and 1 that represents motor power in percent.
   */
    public ExampleCommand(ExampleSubsystem subsystem, DoubleSupplier power) {
      m_subsystem = subsystem;
      m_powerSupplier = power;

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
    }

    //Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // spins the motor at the desired power
      m_subsystem.spinMotor(m_powerSupplier.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {m_subsystem.spinMotor(0);} //Stops the motor once the command has ended

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
