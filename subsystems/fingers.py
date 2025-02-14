from rev import SparkMax, SparkMaxConfig, SparkBase
from commands2 import RunCommand, Subsystem


class Fingers(Subsystem):
    def __init__(self, amp_limit: int = 60):
        self.left_motor = SparkMax(30, SparkMax.MotorType.kBrushless)
        self.right_motor = SparkMax(31, SparkMax.MotorType.kBrushless)
        motor_config = SparkMaxConfig().smartCurrentLimit(amp_limit)
        self.left_motor.configure(
            motor_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )
        # motor_config_right = (
        #     SparkMaxConfig().smartCurrentLimit(amp_limit).follow(self.left_motor, True)
        # )
        self.right_motor.configure(
            motor_config,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )

    def set_motors(self, power: float) -> None:
        power = 1 if power > 1 else -1 if power < -1 else power
        self.left_motor.set(power)
        self.right_motor.set(-power)

    def stop(self) -> RunCommand:
        return RunCommand(lambda: self.set_motors(0), self)

    def intake(self) -> RunCommand:
        return RunCommand(lambda: self.set_motors(0.25), self)

    def score(self) -> RunCommand:
        return RunCommand(lambda: self.set_motors(-0.25), self)
