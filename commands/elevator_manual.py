from typing import Callable

from commands2 import Command

from wpimath.filter import SlewRateLimiter


from subsystems.elevator import Elevator


class ElevatorManual(Command):
    # set at initialize, so value here does not matter
    setpoint: float = 0.0

    def __init__(
        self,
        elevator: Elevator,
        get_power: Callable[[], float],
        percent_rate_limit: float = 1.0,
    ):
        super().__init__()
        self.elevator = elevator
        self.get_power = get_power
        self.limiter = SlewRateLimiter(percent_rate_limit)
        self.addRequirements(elevator)
        self.setName("Elevator Manual")

    def initialize(self) -> None:
        self.setpoint = self.elevator.get_height()
        self.limiter.reset(self.setpoint)
        return super().initialize()

    def execute(self) -> None:
        self.setpoint += (
            self.limiter.calculate(self.get_power()) * self.elevator.max_height
        )

        self.elevator.set_setpoint(self.setpoint)
