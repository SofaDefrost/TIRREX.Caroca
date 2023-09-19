from params import Parameters
from scripts.utils.basebeam import BaseBeam
from scripts.utils.basecosserat import BaseCosserat
from splib3.numerics import Quat, Vec3
import numpy as np
import Sofa.Core
import Sofa.constants.Key as Key
from scripts.utils import addArticulationCenter


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

    def __init__(self, modelling, simulation, name='Caroca', position=[0, 0, 0, 0, 0, 0, 1], cableModel='beam', inverse=False):
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
            pulley = self.pulleys.addChild("Pulley" + str(i))
            pulley.addObject("MechanicalObject", position=[a[i], 0], template='Vec1')
            pulley.addObject('RestShapeSpringsForceField', points=[0], stiffness=1e3, angularStiffness=1e3)
            pulley.addObject('ArticulatedHierarchyContainer')

            rigid = pulley.addChild('Rigid')
            rigid.addObject('MechanicalObject', template='Rigid3',
                             position=[position[i]+[0, 0, 0, 1]] * 3, showObject=True,
                             showObjectScale=0.1,
                             showIndices=False, showIndicesScale=0.1)
            rigid.addObject('ArticulatedSystemMapping', indexInput2=[0, 0, 1, 1, 2, 2, 3, 3][i],
                            input1=pulley.MechanicalObject.getLinkPath(),
                            input2=self.structure.MechanicalObject.getLinkPath(),
                            output=rigid.MechanicalObject.getLinkPath())

            centers = rigid.addChild('ArticulationCenters')
            addArticulationCenter(centers, 'CenterY', 0, 1, position[i], [0, 0, 0], 0, 0, 1, [0, 1, 0], 0)
            addArticulationCenter(centers, 'CenterZ', 1, 2, [0, -0.15, 0], [0, 0, 0], 0, 0, 1, [0, 0, 1], 1)

            visual = rigid.addChild('Visual1')
            visual.addObject('MeshSTLLoader', filename='mesh/pulley1.stl', scale=0.02, translation=[position[i][j]+[0, -0.25, 0][j] for j in range(3)],
                             rotation=[0, 0, 180])
            visual.addObject('OglModel', src='@MeshSTLLoader')
            visual.addObject('RigidMapping', src='@MeshSTLLoader', index=0)

            visual = rigid.addChild('Visual2')
            visual.addObject('MeshSTLLoader', filename='mesh/pulley2.stl', scale=0.02, translation=[0, -0.25, 0],
                             rotation=[0, 90, 180])
            visual.addObject('OglModel', src='@MeshSTLLoader')
            visual.addObject('RigidMapping', src='@MeshSTLLoader', index=1)

            visual = rigid.addChild('Visual3')
            visual.addObject('MeshSTLLoader', filename='mesh/pulley3.stl', scale=0.02, translation=[0, -0.09, 0],
                             rotation=[0, 90, 180])
            visual.addObject('OglModel', src='@MeshSTLLoader')
            visual.addObject('RigidMapping', src='@MeshSTLLoader', index=2)

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

        pulleyId = [0, 2, 4, 6, 1, 3, 5, 7]
        structureId = [0, 1, 2, 3, 0, 1, 2, 3]
        cableLengths = [0] * nbCables
        for i in range(nbCables):
            positionPulley = np.copy(self.pulleys.getChild('Pulley' + str(pulleyId[i])).Rigid.MechanicalObject.position.value[2])
            positionPulley[1] += -0.15
            direction = [positionStructure[structureId[i]][j] + positionPulley[j] - positionBase[i][j] for
                         j in range(3)]
            v = Vec3(direction)
            cableLengths[i] = v.getNorm()
            q = Quat.createFromVectors(v, Vec3([1., 0., 0.]))
            positionBase[i][3:7] = list(q)

        if self.cableModel == 'beam':  # TODO: remove once we have the sliding actuator
            self.structure.addData(name='speed', type='float', help='cable deployment speed', value=0)
            self.structure.addData(name='displacement', type='float', help='cable deployment displacement', value=0)

        for i in range(nbCables):
            if self.cableModel == 'cosserat':
                beam = BaseCosserat(self.modelling, self.simulation, self.params.cable, name='Cable' + str(i),
                                    positionBase=positionBase[i], length=cableLengths[i])
            else:
                beam = BaseBeam(self.modelling, self.simulation, self.params.cable, name='Cable' + str(i),
                                    positionBase=positionBase[i], length=cableLengths[i])

            if self.cableModel == 'beam':  # TODO: remove once we have the sliding actuator
                beam.node.speed.setParent(self.structure.speed.getLinkPath())
                beam.node.displacement.setParent(self.structure.displacement.getLinkPath())

            beam.deformable.addObject('RestShapeSpringsForceField', name="pulley",
                                         points=beam.node.indexExtremity.value-1,
                                         stiffness=1e12, angularStiffness=1e12)

            distance = beam.base.addChild('Distance' + str(i))
            self.corners.addChild(distance)
            distance.addObject('MechanicalObject', template='Rigid3', rest_position=[0, 0, 0, 0, 0, 0, 1])
            distance.addObject('RestShapeSpringsForceField', points=0, stiffness=2e12, angularStiffness=0,
                               drawSpring=True)
            distance.addObject('RigidDistanceMapping',
                               input1=self.corners.MechanicalObject.getLinkPath(),
                               input2=beam.base.MechanicalObject.getLinkPath(),
                               output=distance.MechanicalObject.getLinkPath(), first_point=i, second_point=0)

    def addController(self):
        self.structure.addObject(CarocaController(self.structure))


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    Caroca(modelling, simulation, cableModel='beam')

    rootnode.addObject('VisualGrid', size=10, nbSubdiv=100)
