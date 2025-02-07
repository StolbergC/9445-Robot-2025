from webbrowser import get

from rev import SparkMax, SparkMaxConfig, SparkBase

from phoenix6.hardware import CANcoder
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.signals import SensorDirectionValue

from commands2 import Subsystem, RunCommand, WrapperCommand

from ntcore import NetworkTable, NetworkTableInstance, EventFlags, Event, ValueEventData

from wpilib import DigitalInput


class Climber(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = SparkMax(26, SparkBase.MotorType.kBrushless)
        self.motor_config = (
            SparkMaxConfig()
            .smartCurrentLimit(35)
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        )

        self.motor.configure(
            self.motor_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )
        self.bottom_limit = DigitalInput(1)

    def climb(self):
        return (
            RunCommand(lambda: self.motor.set(1), self)
            .onlyWhile(self.bottom_limit.get)
            .withName("Climb")
        )

    def reverse(self):
        return RunCommand(lambda: self.motor.set(-0.5), self).withName("Reverse")
