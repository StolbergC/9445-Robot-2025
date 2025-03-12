from ntcore import NetworkTableInstance
from rev import SparkBaseConfig, SparkMax, SparkMaxConfig, SparkBase
from commands2 import InstantCommand, RunCommand, Subsystem, WrapperCommand
from wpimath.units import amperes


class Fingers(Subsystem):
    def __init__(self, amp_limit: int = 30):
        self.left_motor = SparkMax(31, SparkMax.MotorType.kBrushless)
        self.right_motor = SparkMax(30, SparkMax.MotorType.kBrushless)
        motor_config = (
            SparkMaxConfig()
            .smartCurrentLimit(amp_limit)
            .setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        )
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

        self.nettable = NetworkTableInstance.getDefault().getTable("000Fingers")

    def periodic(self) -> None:
        self.nettable.putNumber("Amps", self.get_current())
        return super().periodic()

    def set_motors(self, power: float) -> None:
        power = 1 if power > 1 else -1 if power < -1 else power
        self.left_motor.set(power * 1.5)
        self.right_motor.set(-power)

    def get_current(self) -> amperes:
        return max(
            self.left_motor.getOutputCurrent(), self.right_motor.getOutputCurrent()
        )

    def stop(self) -> InstantCommand:
        return InstantCommand(lambda: self.set_motors(0), self)

    def intake(self, is_coral: bool = True) -> WrapperCommand:
        return (
            RunCommand(lambda: self.set_motors(0.65), self)
            .onlyWhile(
                lambda: is_coral or self.get_current() > 25  # TODO: This is a guess
            )
            .withName(f"Intaking {'coral' if is_coral else 'algae'}")
        )

    def score(self) -> WrapperCommand:
        return RunCommand(lambda: self.set_motors(-0.65), self).withName("Score")
