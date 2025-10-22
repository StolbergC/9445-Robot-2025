from commands.elevator_height import Elevator, ElevatorHeight


class ElevatorL3(ElevatorHeight):
    height = 153

    # inheritance covers the rest in this case. The only behavioral change is height
    def __init__(self, elevator: Elevator):
        super().__init__(elevator)
        self.setName("Elevator L3")
