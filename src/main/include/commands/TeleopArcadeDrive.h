// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TeleopArcadeDrive
    : public frc2::CommandHelper<frc2::CommandBase, TeleopArcadeDrive> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  TeleopArcadeDrive(Drivetrain* subsystem, std::function<double()> xaxisSpeed, std::function<double()> zaxisRotate);
  void Execute() override;
 private:
  Drivetrain* m_drive;
  std::function<double()> m_xAxisSpeed;
  std::function<double()> m_zAxisRotate;
};
