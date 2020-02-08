package frc.robot;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

public class CANSparkMaxSmartVelocity extends CANSparkMax {
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private CANPIDController m_pidController;

    public CANSparkMaxSmartVelocity(int deviceID, MotorType type) {
        super(deviceID, type);
        restoreFactoryDefaults();
        setIdleMode(IdleMode.kCoast);

        // PID coefficients
        kP = 0.1;
        kI = 1e-4;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        m_pidController = getPIDController();

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public void set(double rpm) {
        m_pidController.setReference(rpm, ControlType.kSmartVelocity);
    }
}
