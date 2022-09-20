package org.firstinspires.ftc.robot.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robot.commands.ExampleCommand;
import org.firstinspires.ftc.robot.subsystems.ExampleSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Driver Controlled")
// @Disabled
public class TeleOp extends CommandOpMode {

    private ExampleSubsystem m_subsystem = ExampleSubsystem.getInstance();
    private GamepadEx m_driverOp;

    @Override
    public void initialize() {

        //initalize subsystems

        m_subsystem.init(hardwareMap);

        m_driverOp = new GamepadEx(gamepad1);


        register(m_subsystem);

        //set default commands for each subsystem
        m_subsystem.setDefaultCommand(new ExampleCommand(m_subsystem));

        configureButtonBindings();

    }

    private void configureButtonBindings() {
        // set what happens when a button is pressed
        new GamepadButton(m_driverOp, GamepadKeys.Button.A).toggleWhenPressed(new ExampleCommand(m_subsystem));

    }


    // modifies the controller output to make it less sensitive
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