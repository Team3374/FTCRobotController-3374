package org.firstinspires.ftc.robotcontroller.external.broncexamples.differentialexamplecode.subsystems;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.broncexamples.differentialexamplecode.Constants;
import org.firstinspires.ftc.robotcontroller.external.broncexamples.differentialexamplecode.Ports;


public class DrivetrainSubsystem extends SubsystemBase {

    private static DrivetrainSubsystem my_instance = null;

    public static synchronized DrivetrainSubsystem getInstance() {
        if (my_instance == null) {
            my_instance = new DrivetrainSubsystem();
        }
        return my_instance;
    }

    private DifferentialDrive m_drive;

    private DifferentialOdometry m_robotOdometry;

    private OdometrySubsystem m_odometry;

    private MotorEx ld, rd;

    private Encoder m_left, m_right;


    private double WHEEL_DIAMETER;

    /**
     * Creates a new DriveSubsystem.
     */
    public DrivetrainSubsystem() {

    }

    public void init(HardwareMap hardwareMap) {
        ld = new MotorEx(hardwareMap, Ports.L_DRIVE);
        rd = new MotorEx(hardwareMap, Ports.R_DRIVE);

        m_left = ld.encoder;
        m_right = rd.encoder;

        WHEEL_DIAMETER = Constants.DriveConstants.WHEEL_DIAMETER;

        m_drive = new DifferentialDrive(
                ld,
                rd);

        m_robotOdometry = new DifferentialOdometry(() -> m_left.getDistance(), () -> m_right.getDistance(), 18.0);
        m_odometry = new OdometrySubsystem(m_robotOdometry);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param forward the commanded forward movement
     * @param turn the commanded rotational movement
     */
    public void driveArcade(double forward, double turn) {
        m_drive.arcadeDrive(forward, turn);

    }
    public void driveTank(double rightSpeed, double leftSpeed){
        m_drive.tankDrive(rightSpeed, leftSpeed);
    }




    public double getLeftEncoderVal() {
        return m_left.getPosition();
    }

    public double getLeftEncoderDistance() {
        return m_left.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public double getRightEncoderVal() {
        return m_right.getPosition();
    }

    public double getRightEncoderDistance() {
        return m_right.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public void resetEncoders() {
        m_left.reset();
        m_right.reset();
    }

    public double getAverageEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
    }

}
