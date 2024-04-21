import abc
import json
from dataclasses import asdict, dataclass


class Command(abc.ABC):
    def asmsg(self):
        return json.dumps(asdict(self))


@dataclass(frozen=True)
class MoveToXYZ(Command):
    position: list[float]
    action: str = "move-to-xyz"
