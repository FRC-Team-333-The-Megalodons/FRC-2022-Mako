package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

public class LimitSwitch {
    
  private DigitalInput m_limitSwitch;
  private boolean m_hasLimitSwitchBeenPressed;
  private Joystick m_joystick;
    
  public LimitSwitch(Joystick joystick_, DigitalInput limitSwitch_)
  {
      m_limitSwitch = limitSwitch_;
      m_joystick = joystick_;
      m_hasLimitSwitchBeenPressed = false;
  }

  public boolean isPhysicalSwitchPressed()
  {
      return !m_limitSwitch.get();
  }

  public boolean get()
  {
      return get(false);
  }

  public boolean get(boolean isFireButtonPressed)
  {
      // This implements a "latching" mechanism, where once it's been pressed, even if it flutters,
      //  this function will continue returning true (until the trigger is pressed)
      if (isPhysicalSwitchPressed()) {
        m_hasLimitSwitchBeenPressed = true;
      }
  
      if (isFireButtonPressed) {
        m_hasLimitSwitchBeenPressed = false;
      }

      return m_hasLimitSwitchBeenPressed;
  }

  public boolean shouldIgnoreLimitSwitch()
  {
      return m_joystick.getThrottle() >= 0.7;
  }


public static class TimedLimitSwitch
{
  private DigitalInput m_limitSwitch;
  private long m_lastTriggerTime = 0;
  private static final long TRAIL_TIME_MS = 1000;

  public TimedLimitSwitch(DigitalInput limitSwitch)
  {
    m_limitSwitch = limitSwitch;
  }

  public boolean isPhysicalLimitSwitchPressed()
  {
    return !m_limitSwitch.get();
  }

  public void update()
  {
    if (isPhysicalLimitSwitchPressed()) {
      m_lastTriggerTime = System.currentTimeMillis();
    }
  }

  public boolean get()
  {
    update();
  
    if (System.currentTimeMillis() - m_lastTriggerTime <= TRAIL_TIME_MS) {
      return true;
    }

    return isPhysicalLimitSwitchPressed();
  }
}

}
