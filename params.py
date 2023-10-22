from dataclasses import dataclass

# Units are m, kg, s


@dataclass
class CableParameters:

    youngModulus: float = 1.205e11
    poissonRatio: float = 0.499
    radius: float = 0.004
    density: float = 7.850e3
    maxDisplacement: float = 0.3
    nbSections: int = 80
    length: float = 5


@dataclass
class PulleyParameters:

    radius: float = 0.075
    shift: float = 0.2


@dataclass
class PlatformParameters:
    side: float = 1
    mass: float = 2


@dataclass
class StructureParameters:
    height: float = 2.7
    width: float = 2.7
    length: float = 5.7
    thickness: float = 0.2


@dataclass
class Parameters:

    cable: CableParameters = CableParameters()
    pulley: PulleyParameters = PulleyParameters()
    platform: PlatformParameters = PlatformParameters()
    structure: StructureParameters = StructureParameters()





