package frc.robot.utils;

import java.util.ArrayDeque;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotUtils {


public static class NavXGyro {
    public static final int GYRO_HISTORY_LENGTH_MS = 200;

    AHRS m_ahrs;
    double m_lastResetAngle = 0.0;
    History m_history;
    double m_multiplier = 1.0;

    public NavXGyro(SPI.Port port) {
        m_ahrs = new AHRS(port);
        m_ahrs.reset();
        m_ahrs.resetDisplacement();
        do {
            m_lastResetAngle = m_ahrs.getAngle();
        } while (m_lastResetAngle == 0.0);

        m_history = new History(GYRO_HISTORY_LENGTH_MS, false);
        m_history.appendToHistory(m_lastResetAngle);
    }

    public AHRS getAHRS()
    {
        return m_ahrs;
    }

    public void reset() {
        // For some reason, resetting on AHRS doesn't actually work;
        // instead, just do it manually ourselves.
        // m_ahrs.reset(); m_ahrs.resetDisplacement();

        // avoid infinite loops due to possible hardware failure
        final int MAX_ATTEMPTS = 1000;
        int num_attempts = 0;
        do {
            m_lastResetAngle = m_ahrs.getAngle();
            ++num_attempts;
        } while (m_lastResetAngle == 0.0 
                 && num_attempts < MAX_ATTEMPTS);
        m_history.clear();
    }

    public double getAngle() {
        return getAngle(true);
    }

    public double getAngle(boolean record_to_history) {
        double angle = m_ahrs.getAngle() - m_lastResetAngle;
        if (record_to_history) {
            m_history.appendToHistory(angle);
        }
        return angle;
    }

    public double getAverageAngle() {
        getAngle(); // Append to history.
        return m_history.getHistoryAverage();
    }

    // NOTE: Does not save to history! This function is only for diagnostic
    // purposes.
    public double getRawAngle() {
        return getAngle(false);
    }

    // NOTE: Does not save to history! This function is only for diagnostic
    // purposes.
    public double getRawNativeAngle() {
        return m_ahrs.getAngle();
    }

    public double getCurrentOffset() {
        return m_lastResetAngle;
    }

    public void setMultiplier(double multiplier) {
        m_multiplier = multiplier;
    }

    public double getMultiplier() {
        return m_multiplier;
    }
}

private static class History {
    History(int historyLenMs, boolean useAbsValue) {
        m_historyLenMs = historyLenMs;
        m_history = new ArrayDeque<TimeEntry>();
        m_useAbsValue = useAbsValue;
    }

    public void appendToHistory(double magnitude) {
        long now = System.currentTimeMillis();
        while (m_history.peekFirst() != null && (now - m_history.peekFirst().time) > m_historyLenMs) {
            m_history.pollFirst();
        }
        m_history.addLast(new TimeEntry(now, m_useAbsValue ? Math.abs(magnitude) : magnitude));
    }

    public double getHistoryAverage() {
        double sum = 0.0;
        if (m_history.size() < 1) {
            return sum;
        }
        for (TimeEntry e : m_history) {
            sum += e.magnitude;
        }

        return sum / m_history.size();
    }

    /*
    public double getHistoryAverageWithSalt(double salt) {
        int size = 0;
        size = m_history.size();
        return (getHistoryAverage() * size + salt) / (size + 1);
    }
    */

    public void clear() {
        m_history.clear();
    }
    

    private ArrayDeque<TimeEntry> m_history;
    private int m_historyLenMs;
    private boolean m_useAbsValue;
}

private static class TimeEntry {
    public TimeEntry(long time_, double magnitude_) {
        time = time_;
        magnitude = magnitude_;
    }

    public long time;
    public double magnitude;
}



public static class AutonStraightDrive {
    public AutonStraightDrive(DifferentialDrive drive, NavXGyro gyro, DriveTrainEncoder encoder) {
        m_drive = drive;
        m_gyro = gyro;
        m_encoder = encoder;
    }

    
    boolean fuzzyEquals(double a, double b)
    {
        final double EPSILON = 0.05;
        return (Math.abs(a-b) < EPSILON);
    }

    final double SLOWDOWN_POINT = 0.5;

    public boolean periodic(double distanceTarget, double maxSpeed)
    {
        double speed = 0.0;
        if (fuzzyEquals(m_encoder.averageEncoderPosition(), distanceTarget))
        {
            m_drive.arcadeDrive(0.0, 0.0);
            return true;
        }
        
        if (m_encoder.averageEncoderPosition() < distanceTarget)
        {
            speed = maxSpeed;
        }
        else
        {
            speed = -maxSpeed;
        }

        // If we're less than half a meter away, cut our speed in half.
        if (Math.abs(m_encoder.averageEncoderPosition() - distanceTarget) < SLOWDOWN_POINT) { 
            speed /= 2.0;
        }

        //speed *= -1;
        //double rotation = 0;
        double rotation = /*-1 *  */ m_gyro.getAngle(false) * m_gyro.getMultiplier();
        SmartDashboard.putNumber("AutoSpeed", speed);
        SmartDashboard.putNumber("AutoRotation", rotation);
        m_drive.arcadeDrive(rotation, speed);
        return false;
    }

    private NavXGyro m_gyro;
    private DifferentialDrive m_drive;
    private DriveTrainEncoder m_encoder;
}


public static class DriveTrainEncoder {
    private RelativeEncoder m_leftEncoder, m_rightEncoder;

    public DriveTrainEncoder(RelativeEncoder left, RelativeEncoder right) {
        m_leftEncoder = left;
        m_rightEncoder = right;
    }

    public void reset()
    {

        m_leftEncoder.setPosition(0.0);
        m_rightEncoder.setPosition(0.0);
    }

    public double getLeftPosition() {
        return m_leftEncoder.getPosition();
    }

    public double getRightPosition() {
        return m_rightEncoder.getPosition();
    }

    public double averageEncoderPosition() {
        return (m_leftEncoder.getPosition() - m_rightEncoder.getPosition()) / 2;
    }
}

public static class ComboBoxItem {
    private String m_label;
    private AutoOption m_value;

    public ComboBoxItem(String label, AutoOption value)
    {
        m_label = label;
        m_value = value;
    }

    public String getLabel() { return m_label; }
    public AutoOption getValue() { return m_value; }

}

public static class AutoOption {
    public AutoOption(int start_state_, int stop_state_)
    {
        start_state = start_state_;
        stop_state = stop_state_;
    }
    public int start_state;
    public int stop_state;
}

}