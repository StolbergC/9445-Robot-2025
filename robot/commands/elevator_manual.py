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
        speed_mult: float = 1,
    ):
        super().__init__()
        self.elevator = elevator
        self.get_power = get_power
        self.speed_mult = speed_mult
        self.setpoint = 0
        # ! require elevator b/c we want to run this with other sequential commands
        self.setName("Elevator Manual")

    def init(self) -> None:
        self.setpoint = self.elevator.get_setpoint()

    def execute(self) -> None:
        setpoint = self.setpoint
        self.setpoint = (
            self.elevator.get_setpoint() + self.get_power() * self.speed_mult
        )
        # if setpoint > self.elevator.max_height:
        # setpoint = self.elevator.max_height
        # if setpoint < 0:
        #     setpoint = 0

        if setpoint != self.setpoint:
            self.elevator.set_setpoint(self.setpoint)
