from commands2 import Command

from wpimath.units import meters

from subsystems.elevator import Elevator


class ResetElevator(Command):
    def __init__(self, elevator: Elevator):
        self.elevator = elevator
        self.addRequirements(elevator)

    def initialize(self):
        self.elevator.reset_position(0)
        self.elevator.set_setpoint(self.elevator.get_height())

    def isFinished(self) -> bool:
        return True
