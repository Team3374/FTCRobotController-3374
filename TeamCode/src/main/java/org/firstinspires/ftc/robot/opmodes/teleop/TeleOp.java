package org.firstinspires.ftc.robot.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robot.Ports;
import org.firstinspires.ftc.robot.commands.Drive_Commands.DefaultDriveCommand;
import org.firstinspires.ftc.robot.commands.Drive_Commands.FieldCentricDriveCommand;
import org.firstinspires.ftc.robot.subsystems.DrivetrainSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Driver Controlled")
// @Disabled
public class TeleOp extends CommandOpMode {

    static final double WHEEL_DIAMETER = 100.0; // millimeters


    private DrivetrainSubsystem m_drive = DrivetrainSubsystem.getInstance();
    private GamepadEx m_driverOp;

    @Override
    public void initialize() {

        m_drive.init(hardwareMap);

        m_driverOp = new GamepadEx(gamepad1);

        register(m_drive);

        m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive,
                () -> modifyAxis(m_driverOp.getLeftY()),
                () -> modifyAxis(m_driverOp.getLeftX()),
                () -> modifyAxis(m_driverOp.getRightX())));

        configureButtonBindings();
        printToTelemetry();

    }

    private void configureButtonBindings() {
//        new GamepadButton(m_driverOp, GamepadKeys.Button.A).toggleWhenPressed(new FieldCentricDriveCommand(m_drive,
//                () -> modifyAxis(m_driverOp.getLeftY()),
//                () -> modifyAxis(m_driverOp.getLeftX()),
//                () -> modifyAxis(m_driverOp.getRightX()),
//                () -> m_drive.getRawHeading()));

        new GamepadButton(m_driverOp, GamepadKeys.Button.Y).toggleWhenPressed(new InstantCommand(
                                                            () -> m_drive.resetGyro()));

    }

    private void printToTelemetry(){
        telemetry.addData("Current Rotation", "m_drive.getHeading()");
        // telemetry.addData("Raw Heading", m_drive.getRawHeading());
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

//    @Override
//    public void runOpMode(){
//        printToTelemetry();
//    }



}