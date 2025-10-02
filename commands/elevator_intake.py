from commands.elevator_height import Elevator, ElevatorHeight


class ElevatorIntake(ElevatorHeight):
    height = 0.80

    # inheritance covers the rest in this case. The only behavioral change is height
    def __init__(self, elevator: Elevator):
        super().__init__(elevator)
        self.setName("Elevator Intake")
