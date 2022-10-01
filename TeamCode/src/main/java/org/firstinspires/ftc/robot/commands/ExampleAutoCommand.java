package org.firstinspires.ftc.robot.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;


/**
 * Command to spin motor in auto
 */
public class ExampleAutoCommand extends SequentialCommandGroup {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    public ExampleAutoCommand() {
      addCommands(new ExampleCommand(ExampleSubsystem.getInstance(), () -> 0.6));
    }
}
