package org.firstinspires.ftc.robotcontroller.external.broncexamples.differentialexamplecode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.external.broncexamples.differentialexamplecode.commands.Drive_Commands.arcadeDriveCommand;
import org.firstinspires.ftc.robotcontroller.external.broncexamples.differentialexamplecode.commands.Drive_Commands.tankDriveCommand;
import org.firstinspires.ftc.robotcontroller.external.broncexamples.differentialexamplecode.subsystems.DrivetrainSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Driver Controlled")
@Disabled
public class TeleOp extends CommandOpMode {

    static final double WHEEL_DIAMETER = 100.0; // millimeters


    private DrivetrainSubsystem m_drive = DrivetrainSubsystem.getInstance();
    private GamepadEx m_driverOp;

    @Override
    public void initialize() {

        //initalize hardware
        m_drive.init(hardwareMap);

        m_driverOp = new GamepadEx(gamepad1);


        register(m_drive);

        m_drive.setDefaultCommand(new arcadeDriveCommand(m_drive,
                () -> modifyAxis(m_driverOp.getLeftY()),
                () -> modifyAxis(m_driverOp.getRightX())));

        configureButtonBindings();

    }

    private void configureButtonBindings() {
        new GamepadButton(m_driverOp, GamepadKeys.Button.A).toggleWhenPressed(new tankDriveCommand(m_drive,
                () -> modifyAxis(m_driverOp.getLeftY()),
                () -> modifyAxis(m_driverOp.getRightX())));

    }


    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.0);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }


}