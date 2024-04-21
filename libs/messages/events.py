import abc
from dataclasses import dataclass


class Event(abc.ABC):
    pass


@dataclass(frozen=True)
class EnvironmentReady(Event):
    action: str = "load-environment"


@dataclass(frozen=True)
class RobotMoved(Event):
    joint_positions: list[float]
    action: str = "move-robot"


@dataclass(frozen=True)
class TaskCompleted(Event):
    action: str = "task-completed"
