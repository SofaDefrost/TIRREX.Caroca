from params import Parameters
import numpy as np
import Sofa.Core
import Sofa.constants.Key as Key
from scripts.pulley import Pulley
from scripts.cable import Cable


class CarocaController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.structure = args[0]

    def onKeypressedEvent(self, event):

        k = event.get("key")
        dt = self.structure.getRoot().dt.value

        if k == Key.plus:
            self.structure.speed.value = 0.07
        elif k == Key.minus:
            self.structure.speed.value = -0.07
        else:
            self.structure.speed.value = 0
        self.structure.displacement.value = self.structure.displacement.value + self.structure.speed.value * dt


class Caroca:
    """
    params:
    cableModel: (string) either cosserat or beam, default is beam
    """

    params = Parameters()

    def __init__(self, modelling, simulation, name='Caroca', position=[0, 0, 0, 0, 0, 0, 1], cableModel='beam',
                 inverse=False):
        self.modelling = modelling
        self.simulation = simulation
        self.name = name
        self.position = position
        self.cableModel = cableModel
        self.inverse = inverse

        self.__addStructure()
        self.__addPulleys()
        self.__addPlatform()
        self.__addDeformableCables()

        if self.cableModel == 'beam':
            self.addController()

        if self.inverse:
            self.__addEffector()

    def __addStructure(self):
        self.structure = self.modelling.addChild('Structure')

        dx = self.params.structure.length / 2. + self.params.structure.thickness
        dy = self.params.structure.height / 2. + self.params.structure.thickness
        dz = self.params.structure.width / 2. + self.params.structure.thickness
        position = [[dx, dy, dz],
                    [dx, dy, -dz],
                    [-dx, dy, dz],
                    [-dx, dy, -dz],

                    [dx, -dy, dz],
                    [dx, -dy, -dz],
                    [-dx, -dy, dz],
                    [-dx, -dy, -dz]]

        self.structure.addObject('MechanicalObject', template='Rigid3',
                                 position=[pos + [0, 0, 0, 1] for pos in position], showObject=False,
                                 showObjectScale=0.1,
                                 showIndices=False, showIndicesScale=0.1)
        self.structure.addObject('FixedConstraint', fixAll=True)

        visuals = self.structure.addChild('Visuals')
        t = self.params.structure.thickness
        max = [
            [-t, -t, -self.params.structure.width - t],
            [-self.params.structure.length - t, -t, t],
            [self.params.structure.length + t, -t, -t],
            [t, -t, self.params.structure.width + t],
            [-t, t, -self.params.structure.width - t],
            [-self.params.structure.length - t, t, t],
            [self.params.structure.length + t, t, -t],
            [t, t, self.params.structure.width + t]
        ]
        for i in range(8):
            visual = visuals.addChild('VisualTB' + str(i))
            visual.addObject('RegularGridTopology', min=[0, 0, 0], max=max[i])
            visual.addObject('OglModel', color=[0.1, 0.1, 0.1, 1])
            visual.addObject('RigidMapping', index=i)

        max = [
            [-t, - self.params.structure.height - t, -t],
            [-t, - self.params.structure.height - t, t],
            [t, - self.params.structure.height - t, -t],
            [t, - self.params.structure.height - t, t]
        ]
        for i in range(4):
            visual = visuals.addChild('Visual' + str(i))
            visual.addObject('RegularGridTopology', min=[0, 0, 0], max=max[i])
            visual.addObject('OglModel', color=[0.1, 0.1, 0.1, 1])
            visual.addObject('RigidMapping', index=i)

    def __addPulleys(self):
        self.pulleys = self.structure.addChild('Pulleys')

        dx = -self.params.structure.thickness
        shift = -self.params.pulley.shift
        position = [[dx, dx, dx + shift],
                    [dx + shift, dx, dx],

                    [dx, dx, -dx - shift],
                    [dx + shift, dx, -dx],

                    [-dx, dx, dx + shift],
                    [-dx - shift, dx, dx],

                    [-dx, dx, -dx - shift],
                    [-dx - shift, dx, -dx]]

        a = [-0.3, -0.3, 0.3, 0.3, 0.3, 0.3, -0.3, -0.3]
        for i in range(8):
            Pulley(self.pulleys, self.structure,
                   position[i], a[i], [0, 0, 1, 1, 2, 2, 3, 3][i], name="Pulley" + str(i))

    def __addPlatform(self):
        self.platform = self.modelling.addChild('Platform')
        self.simulation.addChild(self.platform)
        self.platform.addObject('MechanicalObject', template='Rigid3', position=self.position, showObject=False,
                                showObjectScale=0.1)
        self.platform.addObject('UniformMass', totalMass=self.params.platform.mass)

        self.corners = self.platform.addChild('Corners')
        dx = self.params.platform.side / 2.
        position = [[dx, dx, dx],
                    [dx, dx, -dx],
                    [-dx, dx, dx],
                    [-dx, dx, -dx],

                    [dx, -dx, dx],
                    [dx, -dx, -dx],
                    [-dx, -dx, dx],
                    [-dx, -dx, -dx]]
        self.corners.addObject('MechanicalObject', template='Rigid3', position=[pos + [0, 0, 0, 1] for pos in position],
                               showObject=False, showObjectScale=0.1,
                               showIndices=False, showIndicesScale=0.1)
        self.corners.addObject('RigidMapping', index=0)

        visual = self.platform.addChild('Visual')
        visual.addObject('MeshOBJLoader', name='loader', filename='mesh/cube.obj',
                         scale=0.5 * self.params.platform.side)
        visual.addObject('MeshTopology', src='@loader')
        visual.addObject('OglModel')
        visual.addObject('RigidMapping')

    def __addEffector(self):

        target = self.simulation.getRoot().addChild('EffectorTarget')
        target.addObject('EulerImplicitSolver', firstOrder=True)
        target.addObject('CGLinearSolver', iterations=10, tolerance=1e-5, threshold=1e-5)
        target.addObject('MechanicalObject', template='Rigid3', position=[0.1, -0.1, 0, 0.15, 0, 0, 1], showObject=True,
                         showObjectScale=1)
        target.addObject('UncoupledConstraintCorrection')
        spheres = target.addChild('Spheres')
        spheres.addObject('MechanicalObject', position=[[0.5, 0, 0], [0, 0, 0.5], [0, 0.5, 0]])
        spheres.addObject('SphereCollisionModel', radius=0.1)
        spheres.addObject('RigidMapping')

        self.platform.addObject('PositionEffector', template='Rigid3', indices=0,
                                effectorGoal=target.MechanicalObject.position.getLinkPath(),
                                useDirections=[1, 1, 1, 1, 0, 1])

    def __addDeformableCables(self):

        nbCables = 8
        positionStructure = self.structure.MechanicalObject.position.value
        positionBase = list(np.copy(self.corners.MechanicalObject.position.value))

        if self.cableModel == 'beam':  # TODO: remove once we have the sliding actuator
            self.structure.addData(name='speed', type='float', help='cable deployment speed', value=0)
            self.structure.addData(name='displacement', type='float', help='cable deployment displacement', value=0)

        pulleyId = [0, 2, 4, 6, 1, 3, 5, 7]
        structureId = [0, 1, 2, 3, 0, 1, 2, 3]
        for i in range(nbCables):
            positionPulley = np.copy(
                self.pulleys.getChild('Pulley' + str(pulleyId[i])).Rigid.MechanicalObject.position.value[2])
            positionPulley[1] += -0.15
            direction = [positionStructure[structureId[i]][j] + positionPulley[j] - positionBase[i][j] for
                         j in range(3)]

            beam = Cable(self.modelling, self.simulation, self.corners, i, direction,
                         self.cableModel, positionBase[i], name="Cable" + str(i)).beam

            if self.cableModel == 'beam':  # TODO: remove once we have the sliding actuator
                beam.node.speed.setParent(self.structure.speed.getLinkPath())
                beam.node.displacement.setParent(self.structure.displacement.getLinkPath())

    def addController(self):
        self.structure.addObject(CarocaController(self.structure))


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    Caroca(modelling, simulation, cableModel='beam')

    rootnode.addObject('VisualGrid', size=10, nbSubdiv=100)
