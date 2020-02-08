package frc.robot;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.DriveConstants.RPM_TO_MPS;

public class CANSparkMaxSmartVelocity extends CANSparkMax {
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private CANPIDController m_pidController;
    private LinearFilter inputFilter = LinearFilter.movingAverage(17);


    public CANSparkMaxSmartVelocity(int deviceID, MotorType type) {
        super(deviceID, type);
        restoreFactoryDefaults();
        setIdleMode(IdleMode.kCoast);
        inputFilter.reset();

        // PID coefficients
        kP = 5e-5;
        kI = 1e-6;
        kD = 0;
        kIz = 0;
        kFF = 0.000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;

        // Smart Motion Coefficients
        maxVel = 5700; // rpm
        maxAcc = 2000;

        m_pidController = getPIDController();

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    }

    @Override
    public void set(double speed) {
        double rpm = speed * maxRPM;
        SmartDashboard.putNumber("set RPM: " + getDeviceId(), rpm);
        double calcFilter = inputFilter.calculate(rpm);
        SmartDashboard.putNumber("set Filtered RPM: " + getDeviceId(), calcFilter);
        m_pidController.setReference(rpm, ControlType.kSmartVelocity);
    }
}
