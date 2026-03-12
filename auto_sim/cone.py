from enum import Enum
from dataclasses import dataclass

class ConeColor(Enum):
	BLUE = "blue"
	YELLOW = "yellow"

@dataclass
class Cone:
     x: int
     y: int
     color: ConeColor