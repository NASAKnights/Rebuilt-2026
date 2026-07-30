// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BetterHalfRaise.h"

BetterHalfRaise::BetterHalfRaise(TurretIntake *_turretIntake, Wrist *_wrist) : 
m_turretIntake{_turretIntake}, m_wrist{_wrist} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_turretIntake);
  AddRequirements(m_wrist);
}

// Called when the command is initially scheduled.
void BetterHalfRaise::Initialize() {
  m_timer.Restart();
  m_turretIntake->Intake();
}

// Called repeatedly when this Command is scheduled to run
void BetterHalfRaise::Execute() {

  // units::time::second_t time = m_timer.Get();

  
  if (m_timer.HasElapsed(units::second_t{1.})){
    if (raise){
      raise = false;
      m_wrist->SetAngle(3.0);
    }
    else if (!raise){
      raise = true;
      m_wrist->SetAngle(35.0);
    }
    m_timer.Restart();
  }
  
}

// Called once the command ends or is interrupted.
void BetterHalfRaise::End(bool interrupted) {
  m_wrist->SetAngle(3.0);
  m_turretIntake->StopIntake();
}

// Returns true when the command should end.
bool BetterHalfRaise::IsFinished() {
  return false;
}
