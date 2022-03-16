package frc.robot.dashboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.RobotUtils.ComboBoxItem;

public class SmartDashboardWrapper
{
    private String m_prefix;
    private static SendableChooser<ComboBoxItem> m_autoPicker = new SendableChooser<ComboBoxItem>();

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

    public void createAutoPicker(ArrayList<ComboBoxItem> items)
    {
        createComboBox(m_autoPicker, items);
    }

    public void createComboBox(SendableChooser<ComboBoxItem> picker, ArrayList<ComboBoxItem> items)
    {
        for (int i = 0; i < items.size(); ++i) {
            ComboBoxItem item = items.get(i);
            if (i == 0) {
                picker.setDefaultOption(item.getLabel(), item);
            } else {
                picker.addOption(item.getLabel(), item);
            }
        }

        SmartDashboard.putData("AutoMode", picker);
    }
    
    public int getAutoSelection()
    {
        return m_autoPicker.getSelected().getValue();
    }
}