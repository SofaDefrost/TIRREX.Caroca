from dataclasses import dataclass, field
from math import pi


# Units are m, kg, s


@dataclass
class CableParameters:
    youngModulus: float = 1.205e11
    poissonRatio: float = 0.499
    radius: float = 0.004
    density: float = 7.850e3
    nbSections: int = 80
    length: float = 5


@dataclass
class PulleyParameters:
    radius: float = 0.075
    shift: float = 0.2


@dataclass
class PlatformParameters:
    side: float = 1
    mass: float = 500


@dataclass
class StructureParameters:  # default values correspond to Caroca structure
    height: float = 2.7
    width: float = 2.7
    length: float = 5.7

    thickness: float = 0.2  # for Caroca structure

    # Orientation of each pulley in radian
    pulleysorientations: list = field(default_factory=lambda: [- 0.3 - pi, - 0.3 - pi,
                                                               0.3 + pi, 0.3 + pi,
                                                               -0.3, -0.3,
                                                               0.3, 0.3])
    # Position of the pulley on the structure ('up' or 'down')
    pulleysUD: list = field(default_factory=lambda: ['up', 'up', 'up', 'up',
                                                     'up', 'up', 'up', 'up'])
    # For each pulley, connect the cable to the corresponding corner of the platform
    # (see the README.md for the corresponding numbering)
    cornersOrder: list = field(default_factory=lambda: [0, 1, 2, 3, 4, 5, 6, 7])


@dataclass
class Parameters:
    cable: CableParameters = CableParameters()
    pulley: PulleyParameters = PulleyParameters()
    platform: PlatformParameters = PlatformParameters()
    structure: StructureParameters = StructureParameters()

    velocity: float = 0.5  # m/s
