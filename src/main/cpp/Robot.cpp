// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include "Drivetrain.h"
#include "PS4Controller.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    m_drive.ResetEncoders();
  }

  void RobotPeriodic() override
  {
    m_drive.UpdateSmartdash();
    frc::SmartDashboard::PutNumber("m_state value", m_state);
  }

  void AutonomousInit() override
  {
    m_autoTimer.Start();
    m_drive.ResetEncoders();
    m_state = 0;
  }

  void AutonomousPeriodic() override
  {
    switch (m_state)
    {

    case 0:
    {

      m_drive.Drive(0.0, -0.1);
      if (m_autoTimer.Get() > 1.0) //1.2
      {
        m_state++;
      }
      break;
    }

    case 1:
    {
      m_drive.Drive(-0.2, 0.0);
      if (m_drive.GetEncoderPositionLeft() < -50.0)
      {
        m_state++;
        m_autoTimer.Reset();
        m_autoTimer.Start();
        m_drive.ResetEncoders();
      }
      break;
    }

    case 2:
    {
      frc::SmartDashboard::PutNumber("Timer", m_autoTimer.Get());
      m_drive.Drive(0.0, 0.10);
      if (m_autoTimer.Get() > 0.5)
      {
        m_state++;
        m_autoTimer.Reset();
        m_autoTimer.Start();
      }
      break;
    }

    case 3:
    {
      m_drive.Drive(-0.3, 0.0);
      if (m_drive.GetEncoderPositionLeft() < -25.0)
      {
        m_state++;
        m_autoTimer.Reset();
        m_autoTimer.Start();
      }
      break;
    }

    case 4:
    {
      m_drive.Drive(-0.05, 0.0);
      m_drive.Dropper(0.0, 1.0);
      if (m_autoTimer.Get() > 1.0)
      {
        m_state++;
      }
      break;
    }
    default:
      m_drive.Drive(0.0, 0.0);
    }

    /* case 0:
    {
      double fwd = 0.35;
      double rot = 0.00;
      m_drive.Drive(fwd, rot);
      if (m_autoTimer.Get() > 0.25)
      {
        m_state++;
      }
      break;
    }
    case 1:
    {
      double fwd = 0.00;
      double rot = 0.15;
      m_drive.Drive(fwd, rot);
      if (m_autoTimer.Get() > 0.1)
      {
        m_state++;
      }
      break;
    }
     
    
    default:
      m_drive.Drive(0.0, 0.0);
    }
  */
  }

  void
  TeleopPeriodic() override
  {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    auto forward = m_controller.GetY(frc::GenericHID::kLeftHand);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we p\ull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    auto rotation = m_controller.GetX(frc::GenericHID::kRightHand);
    forward = -Deadbandforward(forward, 0.05);
    rotation = -Deadbandforward(rotation, 0.05);
    m_drive.Drive(forward, rotation);

    auto front = m_controller.GetTriggerAxis(frc::GenericHID::kLeftHand);
    front = Deadbandforward(front, 0.05);
    auto backward = m_controller.GetTriggerAxis(frc::GenericHID::kRightHand);
    backward = Deadbandforward(backward, 0.05);
    m_drive.Climb(front, backward);

    auto speed = m_controller.GetCircleButton();
    auto back = m_controller.GetSquareButton();
    m_drive.Dropper(speed, back);
  }

  double Deadbandforward(double forward, double deadband)
  {
    if ((std::abs(forward)) < deadband)
    {
      return 0.0;
    }
    else if (forward > 0.95)
    {
      return 1.0;
    }
    else if (forward < -0.95)
    {
      return -1.0;
    }
    else
    {
      return forward;
    }
  }

private:
  PS4Controller m_controller{0};
  PS4Controller m_opcontroller{1};
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
  int m_state = 0;
  Timer m_autoTimer;
  Drivetrain m_drive;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
