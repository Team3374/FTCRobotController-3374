package org.firstinspires.ftc.robot.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robot.Constants;
import org.firstinspires.ftc.robot.Ports;


public class DrivetrainSubsystem extends SubsystemBase {

    private static DrivetrainSubsystem my_instance = null;

    public static synchronized DrivetrainSubsystem getInstance() {
        if (my_instance == null) {
            my_instance = new DrivetrainSubsystem();
        }
        return my_instance;
    }

    private MecanumDrive m_drive;

    private MotorEx fld, frd, bld, brd;
    private RevIMU IMU;

    private Encoder m_leftF, m_rightF, m_leftB, m_rightB;

    private double gyroOffset;

    private double WHEEL_DIAMETER;

    /**
     * Creates a new DriveSubsystem.
     */
    public DrivetrainSubsystem() {

    }

    public void init(HardwareMap hardwareMap) {
        fld = new MotorEx(hardwareMap, Ports.FL_DRIVE, Motor.GoBILDA.RPM_312);
        frd = new MotorEx(hardwareMap, Ports.FR_DRIVE, Motor.GoBILDA.RPM_312);
        bld = new MotorEx(hardwareMap, Ports.BL_DRIVE, Motor.GoBILDA.RPM_312);
        brd = new MotorEx(hardwareMap, Ports.BR_DRIVE, Motor.GoBILDA.RPM_312);
        IMU = new RevIMU(hardwareMap);

        fld.setInverted(false);
        bld.setInverted(true);
        frd.setInverted(true);
        brd.setInverted(false);

        IMU.init();

        m_leftF = fld.encoder;
        m_rightF = frd.encoder;
        m_leftB = bld.encoder;
        m_rightB = brd.encoder;



        WHEEL_DIAMETER = Constants.DriveConstants.WHEEL_DIAMETER;

        m_drive = new MecanumDrive(
                fld,
                frd,
                bld,
                brd);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param forward the commanded forward movement
     * @param strafe the commanded strafe movement
     * @param rotation the commanded rotation
     * @param currentRotation current rotation of the robot
     */
    public void fieldCentricDrive(double forward, double strafe, double rotation, double currentRotation) {
        m_drive.driveFieldCentric(forward,strafe,rotation,currentRotation, false);

    }
    public void robotCentricDrive(double forward, double strafe, double rotation){
        m_drive.driveRobotCentric(strafe,forward,rotation, false);
    }

    public double getRawHeading(){
        return IMU.getHeading();
    }

    public void setGyroOffset () {
        gyroOffset = getRawHeading();
    }

    public void resetGyro() {
        IMU.reset();
    }

    public double getHeading() {
        getRawHeading();
        if (gyroOffset == 0){
            return getRawHeading();
        } else if (getRawHeading() - gyroOffset > 360){
            return (getRawHeading() - gyroOffset) - 360;
        } else if (getRawHeading() - gyroOffset < 0) {
            return (getRawHeading() - gyroOffset) + 360;
        } else {
            return getRawHeading() - gyroOffset;
        }
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
