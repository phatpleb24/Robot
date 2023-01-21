// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopArcadeDrive.h"

TeleopArcadeDrive::TeleopArcadeDrive(Drivetrain* subsystem, std::function<double()> xAxisSpeed, std::function<double()> zAxisRotate)
    : m_drive{subsystem}, m_xAxisSpeed{xAxisSpeed}, m_zAxisRotate{zAxisRotate} {
          AddRequirements({subsystem});
      }

void TeleopArcadeDrive::Execute()
{
    m_drive->ArcadeDrive(m_xAxisSpeed()/5, m_zAxisRotate()/5);
}