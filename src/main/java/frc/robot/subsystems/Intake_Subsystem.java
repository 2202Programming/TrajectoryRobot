/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import static frc.robot.Constants.INTAKE_DOWN_SOLENOID_PCM;
import static frc.robot.Constants.INTAKE_PCM_CAN_ID;
import static frc.robot.Constants.INTAKE_UP_SOLENOID_PCM;
import static frc.robot.Constants.MAGAZINE_DOWN_PCM;
import static frc.robot.Constants.MAGAZINE_PCM_CAN_ID;
import static frc.robot.Constants.MAGAZINE_UP_PCM;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWM;


public class Intake_Subsystem extends SubsystemBase {

  // Intake
  Spark intake_spark = new Spark(PWM.INTAKE);
  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(INTAKE_PCM_CAN_ID, INTAKE_UP_SOLENOID_PCM, INTAKE_DOWN_SOLENOID_PCM);
  
  // magazine
  Spark magazine = new Spark(PWM.MAGAZINE);
  DoubleSolenoid magSolenoid = new DoubleSolenoid(MAGAZINE_PCM_CAN_ID, MAGAZINE_UP_PCM, MAGAZINE_DOWN_PCM);
   
  //state variables
  private boolean intakeIsOn = false;

  public Intake_Subsystem() {
    magazineDown(); // must start in down positon
    raiseIntake();  // must start in the up position
  }

  @Override
  public void periodic() {
    // update RPM variables here, because we do controls on them and don't
    // want to have measurement lag.
  }

  public void raiseIntake() {
    // can't have magazine up with intake up, force it down.
    // Magazine goes down faster, so should be ok...
    if (isMagazineUp()) { magazineDown();}
    intakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void lowerIntake() {
    intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isIntakeUp() {
    return (intakeSolenoid.get() == Value.kForward);
  }

  public void intakeOn(double motorStrength) {
    intakeIsOn = true;
    intake_spark.set(motorStrength);
  }

  public boolean intakeIsOn() {
    return intakeIsOn;
  }

  public void intakeOff() {
    intakeIsOn = false;
    intake_spark.set(0);
  }

  public void magazineOn(double motorStrength) {
    magazine.set(motorStrength);
  }

  public void magazineOff() {
    magazine.set(0);
  }

  public void magazineUp() {
    // intake will come down much faster than magazine will go up
    // so no delay here should be OK... famous last words.
    if  (isIntakeUp()) { lowerIntake();}
    magSolenoid.set(Value.kForward);
  }

  public void magazineDown() {
    magSolenoid.set(Value.kReverse);
  }

  public boolean isMagazineUp() {
    return (magSolenoid.get() == Value.kForward);
  }

}
