package org.firstinspires.ftc.robot.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robot.commands.ExampleAutoCommand;
import org.firstinspires.ftc.robot.commands.ExampleCommand;
import org.firstinspires.ftc.robot.subsystems.ExampleSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Driver Controlled")
// @Disabled
public class exampleAuto extends CommandOpMode {

    private ExampleSubsystem m_subsystem = ExampleSubsystem.getInstance();
    private GamepadEx m_driverOp;

    @Override
    public void initialize() {

        //initalize subsystems

        m_subsystem.init(hardwareMap);

        m_driverOp = new GamepadEx(gamepad1);


        register(m_subsystem);

        schedule(new ExampleAutoCommand());

    }
}