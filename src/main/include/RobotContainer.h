// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/Joystick.h"

#include "subsystems/Drivetrain.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  Drivetrain& GetDrive() {return m_drive;};

  frc2::Command* GetAutonomousCommand();

  frc2::Command* TankDriveCommand();

 private:
  // The robot's subsystems and commands are defined here...
  //frc::SendableChooser<frc2::Command*> m_chooser;
  Drivetrain m_drive;
  frc::Joystick m_joystick{0};

  void ConfigureButtonBindings();
};
