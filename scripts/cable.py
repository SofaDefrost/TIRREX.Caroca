from params import Parameters
from scripts.utils.basebeam import BaseBeam
from scripts.utils.basecosserat import BaseCosserat
from splib3.numerics import Quat, Vec3


class Cable:
    """
    Cable object

    modelling: node
    simulation: node
    node: attach node
    index: index of attach point in attach node
    direction:
    positionBase:
    name:
    """

    params = Parameters()

    def __init__(self, modelling, simulation, node, index, direction,
                 cableModel='beam', positionBase=[0, 0, 0], name="Cable"):
        self.modelling = modelling
        self.simulation = simulation
        self.node = node
        self.index = index
        self.cableModel = cableModel
        self.beam = self.__addCable(direction, positionBase, name)

    def __addCable(self, direction, positionBase, name):
        v = Vec3(direction)
        cableLength = v.getNorm()
        q = Quat.createFromVectors(v, Vec3([1., 0., 0.]))
        positionBase[3:7] = list(q)

        if self.cableModel == 'cosserat':
            beam = BaseCosserat(self.modelling, self.simulation, self.params.cable, name=name,
                                positionBase=positionBase, length=cableLength)
        else:
            beam = BaseBeam(self.modelling, self.simulation, self.params.cable, name=name,
                            positionBase=positionBase, length=cableLength)

        beam.deformable.addObject('RestShapeSpringsForceField', name="pulley",
                                  points=beam.node.indexExtremity.value - 1,
                                  stiffness=1e12, angularStiffness=1e12)

        distance = beam.base.addChild('Distance')
        self.node.addChild(distance)
        distance.addObject('MechanicalObject', template='Rigid3', rest_position=[0, 0, 0, 0, 0, 0, 1])
        distance.addObject('RestShapeSpringsForceField', points=0, stiffness=2e12, angularStiffness=0,
                           drawSpring=True)
        distance.addObject('RigidDistanceMapping',
                           input1=self.node.MechanicalObject.getLinkPath(),
                           input2=beam.base.MechanicalObject.getLinkPath(),
                           output=distance.MechanicalObject.getLinkPath(), first_point=self.index, second_point=0)

        return beam


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    load = simulation.addChild('Load')
    load.addObject('MechanicalObject', position=[0, 0, 0, 0, 0, 0, 1], template='Rigid3',
                   showObject=True, showObjectScale=0.1)
    load.addObject('UniformMass', totalMass=10)

    Cable(modelling, simulation, load, 0, [1, 0, 0],
          cableModel='beam', positionBase=[0, 0, 0], name="Cable")
