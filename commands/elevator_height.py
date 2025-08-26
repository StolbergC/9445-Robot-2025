from wpimath.units import meters

from commands2 import Command

from subsystems.elevator import Elevator


class ElevatorHeight(Command):
    height: meters = 0.25

    def __init__(self, elevator: Elevator):
        super().__init__()
        self.elevator = elevator
        self.addRequirements(self.elevator)
        self.setName(f"Elevator{self.height:.2f}")

    def initialize(self) -> None:
        self.elevator.set_setpoint(self.height)

    def isFinished(self) -> bool:
        return self.elevator.at_setpoint()
