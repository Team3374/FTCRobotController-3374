package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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


    public ExampleSubsystem() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void init() {}


}
