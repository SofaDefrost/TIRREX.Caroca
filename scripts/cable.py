from params import Parameters
from scripts.utils.basebeam import BaseBeam
from scripts.utils.basecosserat import BaseCosserat


class Cable:
    """
    Cable object which can be attached to another object at one of its extremity.

    modelling: node
    simulation: node
    positions:
    attachNode: attach node (will attach to the extremity of the cable)
    attachIndex: index of attach point in attach node
    name:
    """

    params = Parameters()

    def __init__(self,
                 modelling, parent,
                 positions, length,
                 attachNode, attachIndex,
                 cableModel='beam', name="Cable"):
        self.modelling = modelling
        self.parent = parent
        self.attachNode = attachNode
        self.attachIndex = attachIndex
        self.cableModel = cableModel
        self.name = name
        self.beam = self.__addCable(positions, length, name)

    def __addCable(self, positions, length, name):

        if self.cableModel == 'cosserat':
            beam = BaseCosserat(self.modelling, self.parent, self.params.cable,
                                name=name, positions=positions, length=length)
        else:
            beam = BaseBeam(self.modelling, self.parent, self.params.cable,
                            name=name, positions=positions, length=length)

        beam.node.addData(name='length', value=length, type='float', help="cable's length")

        distance = beam.rod.addChild('Difference' + self.name)
        self.attachNode.addChild(distance)
        distance.addObject('MechanicalObject', template='Rigid3', position=[0, 0, 0, 0, 0, 0, 0])
        distance.addObject('RestShapeSpringsForceField', points=[0], stiffness=2e12, angularStiffness=0)
        distance.addObject('BeamProjectionDifferenceMultiMapping',
                           input1=self.attachNode.MechanicalObject.linkpath,
                           input2=beam.rod.MechanicalObject.linkpath,
                           interpolationInput2=beam.rod.BeamInterpolation.linkpath,
                           output=distance.MechanicalObject.linkpath,
                           directions=[1, 1, 1, 0, 0, 0, 0],
                           indicesInput1=self.attachIndex)

        return beam


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers
    import params
    from math import pi, cos, sin

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False, rayleighStiffness=0.2)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields"

    length = 2

    load = simulation.addChild('Load')
    load.addObject('MechanicalObject', template='Rigid3',
                   position=[[length, 0, 0, 0, 0, 0, 1],
                             [length, 1, 0, 0, 0, 0, 1]],
                   showObject=True, showObjectScale=0.1)
    load.addObject('UniformMass', totalMass=100)

    cables = simulation.addChild("Cables")

    nbSections = params.CableParameters.nbSections
    dx = length / nbSections

    positions = [[dx * i, 0, 0, 0, 0, 0, 1] for i in range(nbSections + 1)]
    cable = Cable(modelling, cables,
                  positions=positions, length=length,
                  attachNode=load, attachIndex=0,
                  cableModel="beam", name="CableBeam").beam
    cable.base.addObject('FixedConstraint', indices=[0])

    positions = [[dx * i, 1., 0., 0., 0., 0., 1.] for i in range(nbSections + 1)]
    cable = Cable(modelling, cables,
                  positions=positions, length=length,
                  attachNode=load, attachIndex=1,
                  cableModel="cosserat", name="CableCosserat").beam
    cable.base.addObject('FixedConstraint', indices=[0])


