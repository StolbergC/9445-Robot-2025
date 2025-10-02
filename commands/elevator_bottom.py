from commands.elevator_height import Elevator, ElevatorHeight

from wpimath.units import inchesToMeters


class ElevatorBottom(ElevatorHeight):
    height = 0  # this will be set in init it is temporary

    # inheritance covers the rest in this case. The only behavioral change is height
    def __init__(self, elevator: Elevator):
        super().__init__(elevator)
        self.height = inchesToMeters(self.elevator.b)
        self.setName("Elevator Bottom")
