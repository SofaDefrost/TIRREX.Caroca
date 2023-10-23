from params import Parameters
from scripts.utils.basebeam import BaseBeam
from scripts.utils.basecosserat import BaseCosserat


class Cable:
    """
    Cable object

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
        self.beam = self.__addCable(positions, length, name)

    def __addCable(self, positions, length, name):

        if self.cableModel == 'cosserat':
            beam = BaseCosserat(self.modelling, self.parent, self.params.cable,
                                name=name, positions=positions, length=length)
        else:
            beam = BaseBeam(self.modelling, self.parent, self.params.cable,
                            name=name, positions=positions, length=length)

        beam.node.addData(name='length', value=length, type='float', help="cable's length")

        distance = beam.base.addChild('Distance')
        self.attachNode.addChild(distance)
        distance.addObject('MechanicalObject', template='Rigid3', rest_position=[0, 0, 0, 0, 0, 0, 1])
        distance.addObject('RestShapeSpringsForceField', points=0, stiffness=2e12, angularStiffness=0, drawSpring=True)
        distance.addObject('RigidDistanceMapping',
                           input1=self.attachNode.MechanicalObject.getLinkPath(),
                           input2=beam.deformable.MechanicalObject.getLinkPath(),
                           output=distance.MechanicalObject.getLinkPath(),
                           first_point=self.attachIndex, second_point=len(positions) - 2)

        return beam


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers
    from gui import CablesGUI
    import params

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    length = 5

    load = simulation.addChild('Load')
    load.addObject('MechanicalObject', position=[length, 0, 0, 0, 0, 0, 1], template='Rigid3',
                   showObject=True, showObjectScale=0.1)
    load.addObject('UniformMass', totalMass=10)

    cables = simulation.addChild("Cables")

    nbSections = params.CableParameters.nbSections
    dx = length / nbSections
    positions = [[dx * i, 0, 0, 0, 0, 0, 1] for i in range(nbSections + 1)]
    cable = Cable(modelling, cables,
                  positions=positions, length=length,
                  attachNode=load, attachIndex=0,
                  cableModel="beam", name="Cable").beam
    cable.base.addObject('FixedConstraint', indices=[0])

    rootnode.addObject(CablesGUI(cables=cables))
