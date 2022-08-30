package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commands.ExampleCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExampleSubsystem;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_example_Subsystem = ExampleSubsystem.getInstance();

    public RobotContainer(){
        configureButtonBindings();
    }

    private void configureButtonBindings() {}

}
