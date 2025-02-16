from subsystems.claw import Claw
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist


from wpimath.geometry import Rotation2d


def pretty_close(x: float, y: float) -> bool:
    return abs(x - y) < 0.1


def est_working():
    elevator = Elevator(lambda: Rotation2d(0), Rotation2d.fromDegrees(90))
    elevator.set_state(elevator.top_height + 1)
    assert elevator.pid.getGoal().position == elevator.top_height

    elevator.set_state(elevator.bottom_height - 1)
    assert elevator.pid.getGoal().position == elevator.bottom_height

    elevator.get_wrist_angle = lambda: Rotation2d.fromDegrees(-90)
    elevator.set_state(elevator.bottom_height - 1)
    assert (
        elevator.pid.getGoal().position
        == elevator.bottom_height + elevator.wrist_length
    )

    elevator.get_wrist_angle = lambda: Rotation2d.fromDegrees(90)
    elevator.set_state(elevator.top_height + 1)
    assert (
        elevator.pid.getGoal().position == elevator.top_height - elevator.wrist_length
    )

    elevator.encoder.setPosition(elevator.top_height)
    assert pretty_close(elevator.get_position(), elevator.top_height)

    claw = Claw(lambda: Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(85))
    assert claw.set_motor(1) == 0

    claw.get_wrist_angle = lambda: Rotation2d.fromDegrees(0)
    assert claw.set_motor(1) == 1

    wrist = Wrist()
    wrist.get_claw_distance = lambda: 0
    wrist.safe_claw_distance = 5

    wrist.set_state(Rotation2d.fromDegrees(0))
    assert wrist.pid.getGoal().position == 0
    assert wrist.nettable.getBoolean("Safety/Waiting on Claw", False) == False

    wrist.set_state(Rotation2d.fromDegrees(-45))
    assert wrist.pid.getGoal().position == Rotation2d.fromDegrees(-45).radians()
    assert wrist.nettable.getBoolean("Safety/Waiting on Claw", False) == False

    wrist.get_claw_distance = lambda: 10
    wrist.set_state(Rotation2d.fromDegrees(0))
    assert wrist.nettable.getBoolean("Safety/Waiting on Claw", False) == True

    wrist.set_state(Rotation2d.fromDegrees(-45))
    assert wrist.nettable.getBoolean("Safety/Waiting on Claw", False) == True
