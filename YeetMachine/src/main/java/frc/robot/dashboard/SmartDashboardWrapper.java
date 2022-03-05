package frc.robot.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardWrapper
{
    private String m_prefix;
    private String buildKey(String key)
    {
        return m_prefix + "::" + key;
    }
    public SmartDashboardWrapper(Object o)
    {
        String name = o.getClass().getName();
        m_prefix = name.substring(name.lastIndexOf('.')+1);
    }
    public void putBoolean(String key, boolean value)
    {
        key = buildKey(key);
        SmartDashboard.putBoolean(key, value);
    }
    public boolean getBoolean(String key, boolean defaultValue)
    {
        key = buildKey(key);
        if (!SmartDashboard.containsKey(key)) {
            SmartDashboard.putBoolean(key, defaultValue);
        }
        return SmartDashboard.getBoolean(key, defaultValue);
    }

    public void putNumber(String key, double value)
    {
        key = buildKey(key);
        SmartDashboard.putNumber(key, value);
    }
    public double getNumber(String key, double defaultValue)
    {
        key = buildKey(key);
        if (!SmartDashboard.containsKey(key)) {
            SmartDashboard.putNumber(key, defaultValue);
        }
        return SmartDashboard.getNumber(key, defaultValue);
    }

    public void putString(String key, String value)
    {
        key = buildKey(key);
        SmartDashboard.putString(key, value);
    }
    public String getString(String key, String defaultValue)
    {
        key = buildKey(key);
        if (!SmartDashboard.containsKey(key)) {
            SmartDashboard.putString(key, defaultValue);
        }
        return SmartDashboard.getString(key, defaultValue);
    }
    
}