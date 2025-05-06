from wpilib import DriverStation, reportError


class Error:
    def __init__(self, error: Exception):
        if (
            DriverStation.getEventName() != ""
            or DriverStation.getMatchNumber() > 0
            or DriverStation.getMatchType() != DriverStation.MatchType.kNone
        ):
            reportError(str(error))
        else:
            raise error
