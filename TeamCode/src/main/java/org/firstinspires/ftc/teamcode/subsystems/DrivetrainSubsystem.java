package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Ports;


public class DrivetrainSubsystem extends SubsystemBase {

    private final MecanumDrive m_drive;

    private boolean fieldCentric = false;

    private final Encoder m_leftF, m_rightF, m_leftB, m_rightB;

    private final RevIMU gyro;

    private final double WHEEL_DIAMETER;

    /**
     * Creates a new DriveSubsystem.
     */
    public DrivetrainSubsystem(MotorEx fld, MotorEx frd, MotorEx bld, MotorEx brd, RevIMU imu) {
        m_leftF = fld.encoder;
        m_rightF = frd.encoder;
        m_leftB = bld.encoder;
        m_rightB = brd.encoder;

        gyro = imu;

        WHEEL_DIAMETER = Constants.DriveConstants.WHEEL_DIAMETER;

        m_drive = new MecanumDrive(
                fld,
                brd,
                bld,
                brd);

    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param forward the commanded forward movement
     * @param strafe the commanded strafe movement
     * @param rotation the commanded rotation
     */
    public void drive(double forward, double strafe, double rotation, double currentRotation) {
        if (fieldCentric){
            m_drive.driveFieldCentric(strafe,forward,rotation,currentRotation);
        }else {
            m_drive.driveRobotCentric(strafe,forward,rotation);
        }
    }

    public double getRotation() {
        return gyro.getHeading();
    }

    public double getLeftEncoderVal() {
        return m_leftF.getPosition();
    }

    public double getLeftEncoderDistance() {
        return m_leftF.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public double getRightEncoderVal() {
        return m_rightF.getPosition();
    }

    public double getRightEncoderDistance() {
        return m_rightF.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public void setFieldCentric(){
        fieldCentric = true;
    }

    public void setRobotCentric(){
        fieldCentric = false;
    }

    public void resetEncoders() {
        m_leftF.reset();
        m_rightF.reset();
        m_leftB.reset();
        m_rightB.reset();
    }

    public double getAverageEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
    }

}
