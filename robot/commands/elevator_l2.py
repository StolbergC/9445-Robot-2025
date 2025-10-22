from commands.elevator_height import Elevator, ElevatorHeight


class ElevatorL2(ElevatorHeight):
    height = 100.110

    # inheritance covers the rest in this case. The only behavioral change is height
    def __init__(self, elevator: Elevator):
        super().__init__(elevator)
        self.setName("Elevator L2")
