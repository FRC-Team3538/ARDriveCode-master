// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

  Drivetrain::Drivetrain() {
    m_gyro.Reset();

    

    m_DriveL1.ConfigFactoryDefault();
    m_DriveL2.ConfigFactoryDefault();
    m_DriveR1.ConfigFactoryDefault();
    m_DriveR2.ConfigFactoryDefault();


    m_DriveR1.SetInverted(true);
    m_DriveR2.SetInverted(true);

    m_DriveL1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0);
    m_DriveL2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0);
    m_DriveL1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
    m_DriveL2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    m_DriveR1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0);
    m_DriveR2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0);
    m_DriveR1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
    m_DriveR2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    m_DriveL1.SetSensorPhase(false);
    m_DriveL2.SetSensorPhase(false);
    m_DriveR1.SetSensorPhase(false);
    m_DriveR2.SetSensorPhase(false);

// gtfooooo
    m_Climb.SetNeutralMode(NeutralMode::Brake);
    
  }

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
  const double leftOutput = m_leftPIDController.Calculate(
      m_leftEncoder.GetRate(), speeds.left.to<double>());
  const double rightOutput = m_rightPIDController.Calculate(
      m_rightEncoder.GetRate(), speeds.right.to<double>());

  m_leftGroup.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
  
}

void Drivetrain::Drive(double forward, double rotation) {
    m_DriveL1.Set(forward - rotation);
    m_DriveL2.Set(forward - rotation);
    m_DriveR1.Set(forward + rotation);
    m_DriveR2.Set(forward + rotation);
}

void Drivetrain::Climb(double front, double backward) {
  m_Climb.Set(front - backward);
}

void Drivetrain::Dropper(double speed, double back)
{
  m_dropper.Set(speed - back);
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t(m_leftEncoder.GetDistance()),
                    units::meter_t(m_rightEncoder.GetDistance()));
}

// Reset Encoders
void Drivetrain::ResetEncoders()
{
    m_DriveL1.SetSelectedSensorPosition(0);
    m_DriveR1.SetSelectedSensorPosition(0);
    m_DriveL2.SetSelectedSensorPosition(0);
    m_DriveR2.SetSelectedSensorPosition(0);
}

double Drivetrain::GetEncoderPositionLeft()
{
    return m_DriveL1.GetSelectedSensorPosition(0) * kScaleFactor;
}

double Drivetrain::GetEncoderPositionRight()
{
    return m_DriveR1.GetSelectedSensorPosition(0) * kScaleFactor;
}

double Drivetrain::GetEncoderPosition()
{
    return (GetEncoderPositionLeft() + GetEncoderPositionRight()) / 2.0;
}
void Drivetrain::UpdateSmartdash()
{
    frc::SmartDashboard::PutNumber("Encoderposition for Left", GetEncoderPositionLeft());
    frc::SmartDashboard::PutNumber("Encoderposition for Right", GetEncoderPositionRight());
}