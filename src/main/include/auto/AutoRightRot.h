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
#include <wpi/json.h>
#include <memory>
#include <string>
#include <frc2/Timer.h>
#include <units/velocity.h>
#include "AutoInterfaces.h"
/**
 * Represents a differential drive style drivetrain.
 */
class AutoRightRot : public AutoInterface {
 public:
    // Name of this program, used by SmartDash
    static std::string GetName(); 

private:
    int m_state;
    frc2::Timer m_autoTimer;


public:
    // Constructor requires a reference to the RobotMap
    AutoRightRot() = delete;
    ~AutoRightRot();

    // Auto Program Logic
    void Init();
    void Run();
    void UpdateSmartDashNew();
};


