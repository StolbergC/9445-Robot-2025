"""
from subsystems.claw import Claw
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist


from wpimath.geometry import Rotation2d


def pretty_close(x: float, y: float) -> bool:
    return abs(x - y) < 0.1


def test_working():
    elevator = Elevator(lambda: Rotation2d(0))
    elevator.set_state(elevator.top_height + 1)
    assert elevator.pid.getSetpoint() == elevator.top_height

    elevator.set_state(elevator.bottom_height - 1)
    assert elevator.pid.getSetpoint() == elevator.bottom_height

    elevator.get_wrist_angle = lambda: Rotation2d.fromDegrees(-90)
    elevator.set_state(elevator.bottom_height - 1)
    assert elevator.nettable.getBoolean("Safety/Waiting on Wrist", False) == True

    elevator.get_wrist_angle = lambda: Rotation2d.fromDegrees(90)
    elevator.set_state(elevator.top_height + 1)
    assert elevator.nettable.getBoolean("Safety/Waiting on Wrist", False) == True
    assert elevator.motor.getAppliedOutput() == 0

    claw = Claw(
        lambda: Rotation2d.fromDegrees(90),
        Rotation2d.fromDegrees(85),
        Rotation2d.fromDegrees(-85),
    )
    assert claw.set_motor(0.4) == 0

    claw.get_wrist_angle = lambda: Rotation2d.fromDegrees(0)
    assert claw.set_motor(0.4) == 0.4

    wrist = Wrist()
    wrist.get_claw_distance = lambda: 0
    wrist.safe_claw_distance = 5

    wrist.set_state(Rotation2d.fromDegrees(0))
    assert wrist.pid.getGoal().position == 0
    assert (
        wrist.nettable.getBoolean("Safety/Waiting on Claw", False) == False
    )  # False because not way to hit the elevator at 0deg

    wrist.set_state(Rotation2d.fromDegrees(-45))
    assert wrist.pid.getGoal().position == Rotation2d.fromDegrees(-45).radians()
    assert (
        wrist.nettable.getBoolean("Safety/Waiting on Claw", False) == False
    )  # False because will not hit the elevator

    wrist.get_claw_distance = lambda: 10
    wrist.set_state(Rotation2d.fromDegrees(90))
    assert (
        wrist.nettable.getBoolean("Safety/Waiting on Claw", False) == True
    )  # True because at 90 deg will drive into the elevator

    wrist.set_state(Rotation2d.fromDegrees(-45))
    assert (
        wrist.nettable.getBoolean("Safety/Waiting on Claw", False) == False
    )  # False because this will not drive into the elevator with the claw

"""
