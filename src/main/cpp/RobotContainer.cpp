// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/TeleopArcadeDrive.h"
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <Constants.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/RamseteController.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include<frc2/command/InstantCommand.h>

RobotContainer::RobotContainer(){
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  m_drive.SetDefaultCommand(TeleopArcadeDrive(&m_drive, [this] { return -m_joystick.GetRawAxis(1); } , [this] { return m_joystick.GetRawAxis(2); }));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint
  {
    frc::SimpleMotorFeedforward<units::meters>
    {
      DriveConstants::kS, DriveConstants::kV, DriveConstants::kA
    },
    frc::DifferentialDriveKinematics(DriveConstants::trackWidth), 2_V
  };
  
  frc::TrajectoryConfig config(0.5_mps, 0.2_mps_sq);
  config.SetKinematics(frc::DifferentialDriveKinematics(DriveConstants::trackWidth));
  config.AddConstraint(autoVoltageConstraint);

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0_m,0_m,0_deg},
    {/*frc::Translation2d{2_m,0_m}, frc::Translation2d{1_m,1_m}*/},
    frc::Pose2d{100_m, 0_m, 0_deg},
    config
  );


  
  m_drive.resetOdometry(trajectory.InitialPose());
  frc2::RamseteCommand ramseteCommand
  {
    trajectory,
    [this]() {return m_drive.getPose();},
    frc::RamseteController{},
    frc::SimpleMotorFeedforward<units::meters>{DriveConstants::kS, DriveConstants::kV, DriveConstants::kA},
    frc::DifferentialDriveKinematics(DriveConstants::trackWidth),
    [this]() {return m_drive.getWheelSpeed();},
    frc2::PIDController{0, 0, 0},
    frc2::PIDController{0, 0, 0},
    [this](auto left, auto right){m_drive.tankDriveVolts(left, right);},
    {&m_drive}
  };

  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drive.tankDriveVolts(0_V, 0_V); }, {}));
}

frc2::Command* RobotContainer::TankDriveCommand()
{
  return new frc2::InstantCommand([this]{m_drive.tankDriveVolts(2_V,2_V);},{});
}