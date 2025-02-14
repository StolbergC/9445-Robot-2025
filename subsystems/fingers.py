from rev import SparkMax, SparkMaxConfig, SparkBase
from commands2 import RunCommand, Subsystem


class Fingers(Subsystem):
    def __init__(self):
        self.left_motor = SparkMax(30, SparkMax.MotorType.kBrushless)
        # self.right_motor = SparkMax(31, SparkMax.MotorType.kBrushless)
        motor_config = SparkMaxConfig().smartCurrentLimit(60)
        self.left_motor.configure(
            motor_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )
        # motor_config = motor_config.follow(30, True)
        # self.right_motor.configure(
        #     motor_config,
        #     SparkBase.ResetMode.kResetSafeParameters,
        #     SparkBase.PersistMode.kNoPersistParameters,
        # )

    def stop(self) -> RunCommand:
        return RunCommand(lambda: self.left_motor.set(0), self)

    def intake(self) -> RunCommand:
        return RunCommand(lambda: self.left_motor.set(0.25), self)

    def score(self) -> RunCommand:
        return RunCommand(lambda: self.left_motor.set(-0.25), self)
