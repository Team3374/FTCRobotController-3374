package org.firstinspires.ftc.robot.commands.Drive_Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drive;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    /**
     * Creates a new DefaultDrive.
     *  @param subsystem The drive subsystem this command wil run on.
     * @param translationXSupplier   The control input for driving forwards/backwards
     * @param translationYSupplier  The Control input for sideways movement
     * @param rotationSupplier  The control input for turning
     */
    public DefaultDriveCommand(DrivetrainSubsystem subsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        m_drive = subsystem;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_rotationSupplier = rotationSupplier;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.robotCentricDrive(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.robotCentricDrive(0.0, 0.0, 0.0);
    }

}