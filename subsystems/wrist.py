from rev import SparkBaseConfig, SparkMax, SparkLowLevel, SparkMaxConfig, SparkBase
from phoenix6.hardware import CANcoder
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.signals import SensorDirectionValue

from commands2 import Subsystem


class Wrist(Subsystem):
    def __init__(self):
        self.motor = SparkMax(20, SparkBase.MotorType.kBrushless)
        self.motor_config = self.motor.configAccessor
        self.motor_config.smartCurrentLimit(30)
