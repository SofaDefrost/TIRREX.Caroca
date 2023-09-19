import Sofa.Core
import numpy as np
from scripts.utils.baseobject import BaseObject

eps = 1e-3


class BeamController(Sofa.Core.Controller):
    """
    Control the beam deployment/retraction
    """
    # TODO: remove once we have the sliding actuator

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.name = "BeamController"
        self.object = args[0]  # object node
        self.index = args[1]  # current index to deploy
        self.length = args[2]  # segment length
        self.indexInit = self.index
        self.displacement = 0

    def onAnimateBeginEvent(self, event):

        node = self.object.node
        dt = node.getRoot().dt.value

        if node.speed.value > 0 and self.displacement + node.speed.value * dt > node.displacement.value:
            node.speed.value = 0
        if node.speed.value < 0 and self.displacement + node.speed.value * dt < node.displacement.value:
            node.speed.value = 0

        if self.index < 0 or self.index > self.indexInit:  # the whole beam has been deployed or retracted
            node.speed.value = 0  # stop the motion
            self.index = 0 if self.index < 0 else self.indexInit
            return

        speed = node.speed.value
        lengthList = self.object.rod.BeamInterpolation.lengthList

        # Deploy or retract
        if speed > 0 or speed < 0:
            lengths = list(np.copy(lengthList.value))
            lengths[self.index] += node.speed.value * dt
            self.displacement += node.speed.value * dt
            if self.displacement < 0:
                self.displacement = 0

            # Limit deployment
            if node.speed.value > 0 and lengths[self.index] > self.length:
                lengths[self.index] -= node.speed.value * dt
                self.index -= 1

            # Limit retraction
            if node.speed.value < 0 and lengths[self.index] < eps:
                lengths[self.index] -= node.speed.value * dt
                self.index += 1

            lengthList.value = lengths


class BaseBeam(BaseObject):
    """
    Base rod with a retracted part, based on beam theory, and with a visual and a collision model.
    """

    deformabletemplate = 'Rigid3'

    def __init__(self, modelling, simulation, params, length, name='BaseBeam',
                 positionBase=[0, 0, 0, 0, 0, 0, 1], collisionGroup=0):
        super().__init__(modelling, simulation, params, length, name, positionBase, collisionGroup)
        self.__addRod()
        self.addCylinderTopology()
        # self.addCollisionModel()
        self.addVisualModel()

    def __addRod(self):
        self.node = self.modelling.addChild(self.name)
        self.simulation.addChild(self.node)
        self.node.addObject('RequiredPlugin', pluginName=['BeamAdapter'])

        nbSections = self.params.nbSections
        nbPoints = nbSections + 1

        dx = self.length / nbSections
        self.lengthList = [eps] * nbSections + [dx] * nbSections  # half deployed half retracted
        # TODO: remove once we have the sliding actuator
        position = [[eps * i, 0, 0, 0, 0, 0, 1] for i in range(nbPoints)]
        position += [[position[nbSections][0] + dx * i, 0, 0, 0, 0, 0, 1] for i in range(1, nbPoints)]

        indexPairs = [[0, 0]]
        for i in range(nbSections * 2):
            indexPairs += [[1, i]]

        self.base = self.node.addChild('RigidBase')
        self.base.addObject('MechanicalObject', template='Rigid3', position=self.positionBase)

        # The beam
        self.node.addData(name="indexExtremity", type='int', value=len(position) - 1)
        self.deformable = self.node.addChild('Deformable'+self.name)
        self.deformable.addObject('MechanicalObject', template='Rigid3', position=position[1:len(position)],
                                  translation=self.translation, rotation=self.rotation)

        self.rod = self.deformable.addChild('Rod'+self.name)
        self.base.addChild(self.rod)
        self.rod.addObject('EdgeSetTopologyContainer', edges=[[i, i + 1] for i in range(nbSections * 2)])
        self.rod.addObject('MechanicalObject', template='Rigid3', position=position)
        self.rod.addObject('BeamInterpolation', defaultYoungModulus=self.params.youngModulus,
                            dofsAndBeamsAligned=True, straight=True,
                            radius=self.params.radius, crossSectionShape='circular',
                            defaultPoissonRatio=self.params.poissonRatio)
        self.rod.addObject('AdaptiveBeamForceFieldAndMass', computeMass=True,
                            massDensity=self.params.density)

        self.rod.addObject('SubsetMultiMapping', template="Rigid3,Rigid3",
                                                   input=[self.base.MechanicalObject.getLinkPath(),
                                                          self.deformable.MechanicalObject.getLinkPath()],
                                                   output=self.rod.MechanicalObject.getLinkPath(),
                                                   indexPairs=indexPairs)

        # Define velocity of deployment/retraction and adds controller
        # TODO: remove once we have the sliding actuator
        self.node.addData(name='speed', type='float', help='deployment speed', value=0)
        self.node.addData(name='displacement', type='float', help='deployment displacement', value=0)
        self.node.addObject(BeamController(self, nbPoints-2, dx))


# Test scene
def createScene(rootnode):
    from utils.header import addHeader, addSolvers
    import params

    settings, modelling, simulation = addHeader(rootnode)
    rootnode.VisualStyle.displayFlags = ['hideBehavior']
    addSolvers(simulation)

    beam = BaseBeam(modelling, simulation, params.CableParameters, length=2)
    beam.node.RigidBase.addObject('FixedConstraint', indices=0)
