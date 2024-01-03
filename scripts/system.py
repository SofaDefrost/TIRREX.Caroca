from params import Parameters
import numpy as np
from math import floor, sin, cos, pi

from splib3.numerics import vadd, vsub, Quat, Vec3

from scripts.pulley import Pulley
from scripts.cable import Cable
from scripts.support import Support
from gui import CablesGUI


class System:
    """
    params:
    cableModel: (string) either cosserat or beam, default is beam
    """

    params = Parameters()

    def __init__(self, modelling, simulation, name='System',
                 position=[0, 0, 0, 0, 0, 0, 1], cableModel='beam',
                 inverse=False):

        self.modelling = modelling
        self.simulation = simulation
        self.name = name
        self.position = position
        self.cableModel = cableModel
        self.inverse = inverse
        self.cables = None

        self.__addStructure()
        self.__addStructureVisual()
        self.__addPulleys()
        self.__addPlatform()
        self.__addDeformableCables()

        if self.inverse:
            self.__addEffector()

    def __addStructure(self):
        self.structure = self.modelling.addChild('Structure')
        self.simulation.addChild(self.structure)

        dx = self.params.structure.length / 2. + self.params.structure.thickness
        dy = self.params.structure.height / 2. + self.params.structure.thickness
        dz = self.params.structure.width / 2. + self.params.structure.thickness
        position = [[dx, dy, dz],
                    [dx, -dy, dz],
                    [dx, dy, -dz],
                    [dx, -dy, -dz],

                    [-dx, dy, -dz],
                    [-dx, -dy, -dz],
                    [-dx, dy, dz],
                    [-dx, -dy, dz]]

        self.structure.addObject('MechanicalObject', template='Rigid3',
                                 position=[pos + [0, 0, 0, 1] for pos in position], showObject=False,
                                 showObjectScale=0.1,
                                 showIndices=False, showIndicesScale=0.1)
        self.structure.addObject('FixedConstraint', fixAll=True)

    def __addStructureVisual(self):
        visuals = self.structure.addChild('Visuals')

        if self.params.structure.thickness > 0:  # Caroca structure
            t = self.params.structure.thickness
            max = [
                [-t,                                -t, -self.params.structure.width - t],
                [-self.params.structure.length - t, t,  -t],
                [-self.params.structure.length - t, -t, t],
                [-t,                                t,  self.params.structure.width + t],
                [t,                                 -t, self.params.structure.width + t],
                [self.params.structure.length + t,  t,  t],
                [self.params.structure.length + t,  -t, -t],
                [t,                                 t,  -self.params.structure.width - t]
            ]
            for i in range(8):
                visual = visuals.addChild('VisualTB' + str(i))
                visual.addObject('RegularGridTopology', min=[0, 0, 0], max=max[i])
                visual.addObject('OglModel', color=[0.1, 0.1, 0.1, 1])
                visual.addObject('RigidMapping', index=i)

            max = [
                [-t, - self.params.structure.height - t, -t],
                [-t, -self.params.structure.height - t, t],
                [t, - self.params.structure.height - t, t],
                [t, - self.params.structure.height - t, -t]
            ]
            for i in range(4):
                visual = visuals.addChild('Visual' + str(i))
                visual.addObject('RegularGridTopology', min=[0, 0, 0], max=max[i])
                visual.addObject('OglModel', color=[0.1, 0.1, 0.1, 1])
                visual.addObject('RigidMapping', index=i * 2)
        else:

            dx = self.params.structure.length / 2.
            dy = self.params.structure.height / 2.
            dz = self.params.structure.width / 2.
            nbVisuals = 4
            filename = 'support.stl'

            if dz < 2:
                dz = 0
                nbVisuals = 2
                filename = 'doublesupport.stl'

            for i in range(nbVisuals):
                Support(self.modelling,
                        translation=[[dx, dy, dz],
                                     [-dx, dy, dz],
                                     [dx, dy, -dz],
                                     [-dx, dy, -dz]][i],
                        rotation=[0, [180, 0][i % 2], 0],
                        name="SupportVisu" + str(i),
                        filename=filename
                        )

    def __addPulleys(self):
        self.pulleys = self.structure.addChild('Pulleys')

        dx = -self.params.structure.thickness
        shift = -self.params.pulley.shift
        r = self.params.pulley.radius
        dy = self.params.structure.height - dx
        self.positionsPulley = [[dx,          -r,  dx + shift],
                                [dx + shift,   r,  dx],

                                [dx,          -r,  -dx - shift],
                                [dx + shift,   r,  -dx],

                                [-dx,         -r, -dx - shift],
                                [-dx - shift,  r, -dx],

                                [-dx,         -r, dx + shift],
                                [-dx - shift,  r, dx]]

        structure = ['up', 'down', 'up', 'down', 'up', 'down', 'up', 'down']
        for i in range(8):
            if structure[i] != self.params.structure.pulleysUD[i]:
                self.positionsPulley[i][1] += [1, -1][self.params.structure.pulleysUD[i] == "down"] * dy

        a = self.params.structure.pulleysorientations
        self.pulleys.addObject('MechanicalObject', template='Rigid3',
                               position=[self.positionsPulley[i] + list(Quat.createFromAxisAngle([0., 1., 0.], a[i]))
                                         for i in range(8)],
                               showObject=False, showIndices=False)
        self.pulleys.addObject('RigidMapping', rigidIndexPerPoint=list(range(8)), globalToLocalCoords=False)

        for i in range(8):
            Pulley(self.pulleys,
                   angle=0, indexInput2=i, name="Pulley" + str(i))

    def __addPlatform(self):
        self.platform = self.modelling.addChild('Platform')
        self.simulation.addChild(self.platform)
        self.platform.addObject('MechanicalObject', template='Rigid3', position=self.position, showObject=False,
                                showObjectScale=0.1)
        self.platform.addObject('UniformMass', totalMass=self.params.platform.mass)

        self.corners = self.platform.addChild('Corners')
        dx = self.params.platform.side / 2.
        position = [[dx, dx, dx],
                    [dx, -dx, dx],
                    [dx, dx, -dx],
                    [dx, -dx, -dx],

                    [-dx, dx, -dx],
                    [-dx, -dx, -dx],
                    [-dx, dx, dx],
                    [-dx, -dx, dx]]
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
        pulleyRadius = self.params.pulley.radius

        if self.cableModel == 'beam':  # TODO: remove once we have the sliding actuator
            self.structure.addData(name='velocity', type='float', help='cable deployment velocity', value=0)
            self.structure.addData(name='displacement', type='float', help='cable deployment displacement', value=0)

        self.cables = self.simulation.addChild('Cables')
        for i in range(nbCables):

            # Entry position (cable / pulley)
            beginPulley = vadd(positionStructure[i], self.positionsPulley[i])

            # Top position
            shift = Vec3([pulleyRadius, pulleyRadius, 0])
            q = Quat.createFromAxisAngle([0., 1., 0.], self.params.structure.pulleysorientations[i])
            shift.rotateFromQuat(q)
            topPulley = vadd(beginPulley, shift)

            # Center position
            shift = Vec3([pulleyRadius, 0, 0])
            q = Quat.createFromAxisAngle([0., 1., 0.], self.params.structure.pulleysorientations[i])
            shift.rotateFromQuat(q)
            centerPulley = vadd(beginPulley, shift)

            # Cable direction
            direction = Vec3(vsub(positionBase[self.params.structure.cornersOrder[i]], topPulley))
            totalLength = self.params.cable.length
            length1 = direction.getNorm()
            length2 = totalLength - length1 - pi / 2 * pulleyRadius
            direction.normalize()

            nbSections = self.params.cable.nbSections
            dx = totalLength / nbSections

            nbSections1 = floor(length1 / totalLength * nbSections)
            nbSectionsOnPulley = floor((totalLength - length1 - length2) / dx)
            nbSections2 = nbSections - nbSections1 - nbSectionsOnPulley

            positions = [[beginPulley[0],
                          beginPulley[1] - length2 + dx * i,
                          beginPulley[2],
                          0., 0., 0.707, 0.707] for i in range(nbSections2)]

            for j in range(nbSectionsOnPulley):
                angle = -pi / 2 / (nbSectionsOnPulley + 1) * (j + 1)
                qPulley = Quat.createFromAxisAngle([0., 1., 0.], self.params.structure.pulleysorientations[i])
                axis = Vec3([0., 0., 1.]).rotateFromQuat(qPulley)
                axis.normalize()
                q = Quat.createFromAxisAngle(axis, angle)
                pos = vadd(centerPulley, Vec3(vsub(beginPulley, centerPulley)).rotateFromQuat(q))
                q = Quat.createFromAxisAngle(axis, -angle)
                q.rotateFromQuat(qPulley)
                positions += [list(pos) + list(q)]

            q = Quat.createFromVectors(Vec3(direction), Vec3([0., 1., 0.]))
            q.rotateFromQuat(Quat([0., 0., 0.707, 0.707]))
            q.normalize()
            positions += [[topPulley[0] + direction[0] * dx * i,
                           topPulley[1] + direction[1] * dx * i,
                           topPulley[2] + direction[2] * dx * i]
                          + list(q) for i in range(nbSections1)]
            positions += [[topPulley[0] + direction[0] * length1 * 1.001,
                           topPulley[1] + direction[1] * length1 * 1.001,
                           topPulley[2] + direction[2] * length1 * 1.001]
                          + list(q)]

            beam = Cable(self.modelling, self.cables,
                         positions=positions, length=totalLength,
                         attachNode=self.corners, attachIndex=self.params.structure.cornersOrder[i],
                         cableModel=self.cableModel, name="Cable" + str(i)).beam

            slidingpoints = self.pulleys.getChild('Pulley' + str(i)).Rigid.SlidingPoints

            difference = beam.rod.addChild('Difference')
            slidingpoints.addChild(difference)

            nbSlidingPoints = 5
            difference.addObject('MechanicalObject', template='Rigid3', position=[0, 0, 0, 0, 0, 0, 0] * nbSlidingPoints)
            difference.addObject('RestShapeSpringsForceField', points=list(range(nbSlidingPoints)), stiffness=1e12,
                                 angularStiffness=0)
            difference.addObject('BeamProjectionDifferenceMultiMapping', template='Rigid3,Rigid3,Rigid3',
                                 directions=[0, 1, 1, 0, 0, 0, 0],
                                 indicesInput1=list(range(nbSlidingPoints)),
                                 input1=slidingpoints.getMechanicalState().linkpath,
                                 input2=beam.rod.getMechanicalState().linkpath,
                                 interpolationInput2=beam.rod.BeamInterpolation.linkpath,
                                 output=difference.getMechanicalState().linkpath,
                                 draw=False, drawSize=0.1)

            if self.cableModel == 'beam':  # TODO: remove once we have the sliding actuator
                beam.node.velocity.setParent(self.structure.velocity.getLinkPath())
                beam.node.displacement.setParent(self.structure.displacement.getLinkPath())

    def addController(self):
        self.structure.addObject(CablesGUI(self.cables))


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False, rayleighStiffness=0.2)
    rootnode.VisualStyle.displayFlags = "showBehavior showVisualModels showWireframe"

    caroca = System(modelling, simulation, cableModel='beam')
    for i, cable in enumerate(caroca.cables.children):
        cable.RigidBase.addObject('RestShapeSpringsForceField', points=[0], stiffness=1e12)
