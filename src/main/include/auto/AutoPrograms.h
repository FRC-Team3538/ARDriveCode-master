#pragma once

#include <string>

#include <frc/smartdashboard/SmartDashboard.h>

#include "auto/AutoInterfaces.h"
#include <iostream>
#include <frc/smartdashboard/SendableChooser.h>

class AutoPrograms
{

  private:
    // Get a referance to the robotmap

    // Selected Auto Program
    AutoInterface* m_autoProgram;

    // SmartDash Chooser


  public:
    // Constructor requires a reference to the RobotMap
    AutoPrograms() = delete;

    // Choose a program to Initialize
    void Init();

    // Run the selected program
    void Run();
    void SmartDash();
};