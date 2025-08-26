from typing import Callable

from commands2 import Command


from subsystems.elevator import Elevator


"""
This potentially should be refactored to just set the motor values directly and avoid the closed loop control of the system
This will be determined by performance of the system under this if used at all
"""


class ElevatorManual(Command):
    def __init__(
        self,
        elevator: Elevator,
        get_power: Callable[[], float],
        speed_mult: float = 0.1,
    ):
        super().__init__()
        self.elevator = elevator
        self.get_power = get_power
        self.speed_mult = speed_mult
        self.addRequirements(elevator)
        self.setName("Elevator Manual")

    def execute(self) -> None:
        setpoint = self.elevator.get_height() + self.get_power() * self.speed_mult
        if setpoint > self.elevator.max_height:
            setpoint = self.elevator.max_height
        if setpoint < 0:
            setpoint = 0

        self.elevator.set_setpoint(setpoint)
