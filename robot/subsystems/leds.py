from commands2 import Command, RepeatCommand, Subsystem, WaitCommand
from wpilib import AddressableLED, Color, SmartDashboard, Timer
import typing

from math import sin


"""Reactions
>50(?) frames of vision targets seen before enable -> ready to start, change lights
In a scoring position -> Change lights
In an intake position -> Change lights
"""


class Leds(Subsystem):
    # made up
    len: int = 100

    def __init__(self):

        super().__init__()
        self.leds = AddressableLED(9)
        self.leds.setLength(self.len)
        self.leds.setData
        self.buf: list[AddressableLED.LEDData] = [
            AddressableLED.LEDData() for _ in range(self.len)
        ]

        self.timer = Timer()
        self.timer.start()

        self.leds.setData(self.buf)
        self.leds.start()

        DoubleChase(self, Color(255, 0, 0), Color(0, 0, 255), self.len // 2).schedule()

        SmartDashboard.putData(self)

    def periodic(self) -> None:
        self.leds.setData(self.buf)
        self.leds.start()
        return super().periodic()


class SetColor(Command):
    def __init__(self, leds: Leds, color: Color):
        super().__init__()
        self.leds = leds
        self.color = color
        self.addRequirements(self.leds)

    def initialize(self) -> None:
        self.leds.buf = [
            AddressableLED.LEDData(
                int(self.color.red * 255),
                int(self.color.green * 255),
                int(self.color.blue * 255),
            )
            for _ in range(self.leds.len)
        ]

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True


def TwoColorBlink(
    leds: Leds, color1: Color, color2: Color, period: float = 1
) -> Command:
    return RepeatCommand(
        SetColor(leds, color1)
        .andThen(WaitCommand(period / 2))
        .andThen(SetColor(leds, color2))
        .andThen(WaitCommand(period / 2))
    )


def BlinkSolid(leds: Leds, color: Color, delay: float = 0.5) -> Command:
    return (
        SetColor(leds, color)
        .andThen(WaitCommand(delay))
        .andThen(SetColor(leds, Color(0, 0, 0)))
        .andThen(WaitCommand(delay))
        .andThen(SetColor(leds, color))
        .andThen(WaitCommand(delay))
        .andThen(SetColor(leds, Color(0, 0, 0)))
        .andThen(WaitCommand(delay))
        .andThen(SetColor(leds, color))
    )


class Breathe(Command):
    def __init__(self, leds: Leds, color: Color, period: float = 1):
        super().__init__()
        self.leds = leds
        self.color = color
        self.period = period
        self.addRequirements(self.leds)

    def initialize(self) -> None:
        self.leds.timer.reset()
        self.leds.timer.start()

    def execute(self):
        for i in range(self.leds.len):
            t = self.leds.timer.get()
            self.leds.buf[i] = AddressableLED.LEDData(
                int(self.color.red * 255 * abs(sin(t / self.period)) ** 0.9),
                int(self.color.green * 255 * abs(sin(t / self.period)) ** 0.9),
                int(self.color.blue * 255 * abs(sin(t / self.period)) ** 0.9),
            )

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return True


class DoubleChase(Command):
    def __init__(
        self,
        leds: Leds,
        color1: Color,
        color2: Color,
        chase_length: int,
    ):
        super().__init__()
        self.leds = leds
        self.color1 = color1
        self.color2 = color2
        self.addRequirements(self.leds)
        self.index = 0
        self.len = chase_length

    def initialize(self):
        return super().initialize()

    def execute(self):
        end = self.index + self.len
        if end > self.leds.len:
            for i in range(self.index, self.leds.len):
                self.leds.buf[i] = AddressableLED.LEDData(
                    int(self.color1.red * 255),
                    int(self.color1.green * 255),
                    int(self.color1.blue * 255),
                )
            for i in range(0, end - self.leds.len):
                self.leds.buf[i] = AddressableLED.LEDData(
                    int(self.color1.red * 255),
                    int(self.color1.green * 255),
                    int(self.color1.blue * 255),
                )
            for i in range(end - self.leds.len, self.index + 1):
                self.leds.buf[i] = AddressableLED.LEDData(
                    int(self.color2.red * 255),
                    int(self.color2.green * 255),
                    int(self.color2.blue * 255),
                )

        else:
            for i in range(self.index, end):
                self.leds.buf[i] = AddressableLED.LEDData(
                    int(self.color1.red * 255),
                    int(self.color1.green * 255),
                    int(self.color1.blue * 255),
                )
            for i in range(0, self.index + 1):
                self.leds.buf[i] = AddressableLED.LEDData(
                    int(self.color2.red * 255),
                    int(self.color2.green * 255),
                    int(self.color2.blue * 255),
                )
        self.index += 1
        if self.index >= self.leds.len:
            self.index = 0

        return super().execute()

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return True
