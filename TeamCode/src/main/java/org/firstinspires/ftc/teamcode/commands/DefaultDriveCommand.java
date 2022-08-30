package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcontroller.external.ftclibexamples.CommandSample.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_strafe;
    private final DoubleSupplier m_rotation;

    /**
     * Creates a new DefaultDrive.
     *  @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param strafe  The Control input for sideways movement
     * @param rotation  The control input for turning
     */
    public DefaultDriveCommand(DrivetrainSubsystem subsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        m_drive = subsystem;
        m_forward = forward;
        m_strafe = strafe;
        m_rotation = rotation;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(
                m_forward.getAsDouble(),
                m_strafe.getAsDouble(),
                m_rotation.getAsDouble(),
                m_drive.getRotation());
    }

}