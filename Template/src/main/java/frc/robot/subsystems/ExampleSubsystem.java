// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

  //Constant Parameters here can be movred to constsants file
  private final int MOTOR_CANID = 1;
  private final int ENCODER_CANID = 2;

  private final double STATOR_CURRENT_LIMIT = 60.0;
  private final double SUPPLY_CURRENT_LIMIT = 40.0;
  private final double SUPPLY_CURRENT_LIMIT_LOWER_TIME = 2.0;
  private final double SLOT0_KV = 0.11;
  private final double SLOT0_KP = 0.1;
  private final double SLOT0_KI = 0.0;
  private final double SLOT0_KD = 0.0;
  private final double SLOT0_KG = 0.0;
  private final double SLOT0_KA = 0.0;
  private final double MM_VELOCITY = 80.0;
  private final double MM_ACCELERATION = 160.0;
  private final double MM_JERK = 1600.0;

  private final Angle CANCODER_DISCONTINUITY_POINT = Rotations.of(0.0);
  private final Angle CANCODER_MAGNET_OFFSET = Rotations.of(0.0);
  private final SensorDirectionValue CANCODER_SENSOR_DIRECTION = SensorDirectionValue.Clockwise_Positive;

  private final double ROTOR_SENSOR_RATIO = 1.0;
  private final double MECHANISM_SENSOR_RATIO = 1.0;

  //Define Motors and Sensors

  private final TalonFX templateMotor = new TalonFX(MOTOR_CANID, CANBus.roboRIO());
  private final CANcoder templateCanCoder = new CANcoder(ENCODER_CANID);

  //Subsystem Variables

  private Angle templateMotorSetpoint = Rotations.of(0.0);
  private DynamicMotionMagicVoltage motionMagicRequest = new DynamicMotionMagicVoltage(0.0, MM_VELOCITY, MM_ACCELERATION).withJerk(MM_JERK);

  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("Template subsystem");
  private static final DoublePublisher templateMotorPositionPub = table.getDoubleTopic("Motor Position").publish(),
                                        templateMotorSetPointPub = table.getDoubleTopic("Motor SetPoint").publish();

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(CANCODER_DISCONTINUITY_POINT)
      .withMagnetOffset(CANCODER_MAGNET_OFFSET)
      .withSensorDirection(CANCODER_SENSOR_DIRECTION);
    
    templateCanCoder.getConfigurator().apply(canCoderConfig);

    config.CurrentLimits.withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
      .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LIMIT_LOWER_TIME);
    
    config.Slot0.withKV(SLOT0_KV)
      .withKP(SLOT0_KP)
      .withKI(SLOT0_KI)
      .withKD(SLOT0_KD)
      .withKG(SLOT0_KG)
      .withKA(SLOT0_KA);

    config.MotionMagic.withMotionMagicCruiseVelocity(MM_VELOCITY)
      .withMotionMagicAcceleration(MM_ACCELERATION)
      .withMotionMagicJerk(MM_JERK);

    config.Feedback.withFusedCANcoder(templateCanCoder)
      .withRotorToSensorRatio(ROTOR_SENSOR_RATIO)
      .withSensorToMechanismRatio(MECHANISM_SENSOR_RATIO);

    templateMotor.getConfigurator().apply(config);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand(Angle newSetPoint) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          templateMotorSetpoint = newSetPoint;
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    templateMotor.setControl(motionMagicRequest.withPosition(templateMotorSetpoint).withEnableFOC(true));
    templateMotorPositionPub.set(templateMotor.getPosition().getValueAsDouble());
    templateMotorSetPointPub.set(templateMotorSetpoint.in(Units.Degrees));
  }

  @Override
  public void simulationPeriodic() {
    templateMotor.setControl(motionMagicRequest.withPosition(templateMotorSetpoint).withEnableFOC(true));
    templateMotorPositionPub.set(templateMotor.getPosition().getValueAsDouble());
    templateMotorSetPointPub.set(templateMotorSetpoint.in(Units.Degrees));
    // This method will be called once per scheduler run during simulation
  }
}
