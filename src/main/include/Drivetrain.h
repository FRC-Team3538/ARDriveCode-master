// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/math>
#include "ctre/Phoenix.h"
#include "adi/ADIS16470_IMU.h"
/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain();

  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void Drive(double forward, double rotation);
  void UpdateOdometry();
  void UpdateSmartdash();
  void ResetEncoders();
  void ResetGyro();
  double GetEncoderPositionLeft();
  double GetEncoderPositionRight();
  double GetEncoderPosition();
  double GetGyroHeading();
  // nuuuuuuu
  void Climb(double front, double backward);
  void Dropper(double speed, double back);
  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::math::pi};  // 1/2 rotation per second
  double deadband = 0.05;
  double forwardHeading = 0;
  const double kScaleFactor = 0.00042518; //53.1875 / 52896;

 private:
  static constexpr units::meter_t kTrackWidth = 0.381_m * 2;
  static constexpr double kWheelRadius = 0.0508;  // meters
  static constexpr int kEncoderResolution = 4096;

  WPI_TalonFX m_DriveL1{1};
  WPI_TalonFX m_DriveR1{2};
  WPI_TalonFX m_DriveL2{3};
  WPI_TalonFX m_DriveR2{4};
  WPI_VictorSPX m_Climb{5};
  WPI_VictorSPX m_dropper{6};
  frc::SpeedControllerGroup m_leftGroup{m_DriveL1, m_DriveL2};
  frc::SpeedControllerGroup m_rightGroup{m_DriveR1, m_DriveR2};

  frc::Encoder m_leftEncoder{0, 1};
  frc::Encoder m_rightEncoder{2, 3};

  frc2::PIDController m_leftPIDController{1.0, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{1.0, 0.0, 0.0};


 frc::ADIS16470_IMU m_imu{
      frc::ADIS16470_IMU::IMUAxis::kZ,
      frc::SPI::Port::kOnboardCS0,
      frc::ADIS16470CalibrationTime::_4s};


  // Gains are for example purposes only - must be determined for your own
  // robot!
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{1_V, 3_V / 1_mps};
};
