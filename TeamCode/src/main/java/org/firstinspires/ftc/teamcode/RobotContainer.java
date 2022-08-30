package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem m_drive = DrivetrainSubsystem.getInstance();

    public RobotContainer(){
        configureButtonBindings();
    }

    private void configureButtonBindings() {}

}
