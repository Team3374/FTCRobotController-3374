package org.firstinspires.ftc.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robot.Ports;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2020
 * to 2021 season.
 */
public class ExampleSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

    private static ExampleSubsystem my_instance = null;

    public static synchronized ExampleSubsystem getInstance() {
        if (my_instance == null) {
            my_instance = new ExampleSubsystem();
        }
        return my_instance;
    }

    private Motor exampleMoter;


    public ExampleSubsystem() {}

    public void init(HardwareMap hardwareMap){
        // use this function to hardware map and set up hardware
        exampleMoter = new Motor(hardwareMap, Ports.EXAMPLE_MOTOR);

        exampleMoter.setRunMode(Motor.RunMode.RawPower);
    }

    public void spinMotor(double power){
        exampleMoter.set(power);
    }


}
