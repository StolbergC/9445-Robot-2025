from commands.elevator_height import Elevator, ElevatorHeight


class ElevatorBottom(ElevatorHeight):
    # inheritance covers the rest in this case. The only behavioral change is height
    def __init__(self, elevator: Elevator):
        super().__init__(elevator)
        self.height = 0
        self.setName("Elevator Bottom")
