package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.external.ftclibexamples.CommandSample.DefaultDrive;
import org.firstinspires.ftc.robotcontroller.external.ftclibexamples.CommandSample.DriveSubsystem;
import org.firstinspires.ftc.robotcontroller.external.ftclibexamples.CommandSample.GrabStone;
import org.firstinspires.ftc.robotcontroller.external.ftclibexamples.CommandSample.GripperSubsystem;
import org.firstinspires.ftc.robotcontroller.external.ftclibexamples.CommandSample.ReleaseStone;
import org.firstinspires.ftc.teamcode.Ports;
import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Sample TeleOp")
@Disabled
public class TeleOp extends CommandOpMode {

    static final double WHEEL_DIAMETER = 100.0; // millimeters

    private MotorEx fld, frd, bld, brd;
    private RevIMU IMU;
    private DrivetrainSubsystem m_drive;
    private GamepadEx m_driverOp;
    private DefaultDriveCommand m_driveCommand;

    @Override
    public void initialize() {
        fld = new MotorEx(hardwareMap, Ports.FL_DRIVE);
        frd = new MotorEx(hardwareMap, Ports.FR_DRIVE);
        bld = new MotorEx(hardwareMap, Ports.BL_DRIVE);
        brd = new MotorEx(hardwareMap, Ports.BR_DRIVE);
        IMU = new RevIMU(hardwareMap);

        m_drive = new DrivetrainSubsystem(fld, frd, bld, brd, IMU);

        m_driverOp = new GamepadEx(gamepad1);
        m_driveCommand = new DefaultDriveCommand(m_drive,
                () -> m_driverOp.getLeftY(),
                () -> m_driverOp.getLeftX(),
                () -> m_driverOp.getRightX());

        register(m_drive);
        m_drive.setDefaultCommand(m_driveCommand);
    }

}