from scripts.utils import addArticulationCenter
import os
from math import sin, cos, pi, floor
from params import Parameters
import numpy as np
import Sofa

path = os.path.dirname(os.path.realpath(__file__)) + '/../'


class PulleyController(Sofa.Core.Controller):
    """
    Control the pulley visual rotation
    """

    params = Parameters()

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.name = "PulleyController"
        self.pulley = args[0]
        self.cable = args[1]

    def onAnimateBeginEvent(self, event):

        if self.cable is not None:
            position = list(np.copy(self.pulley.getMechanicalState().rest_position.value))
            position[1] = [- self.cable.BeamController.displacement / self.params.pulley.radius]
            self.pulley.getMechanicalState().rest_position.value = position


class Pulley:
    """
    Pulley object
    """

    params = Parameters()

    def __init__(self, node, angle=0., indexInput2=0, name="Pulley"):
        self.__addPulley(node, angle, indexInput2, name)

    def __addPulley(self, node, angle, indexInput2, name):
        self.node = node.addChild(name)
        pulley = self.node
        pulley.addObject("MechanicalObject", position=[angle, 0.], template='Vec1')
        pulley.addObject('StopperConstraint', index=[0], min=-pi/2, max=pi/2)
        pulley.addObject('RestShapeSpringsForceField', name='rssff2', points=[1], stiffness=1e12)
        pulley.addObject('ArticulatedHierarchyContainer')

        r = self.params.pulley.radius

        rigid = pulley.addChild('Rigid')
        rigid.addObject('MechanicalObject', template='Rigid3',
                        position=[0, 0, 0, 0, 0, 0, 1] * 3,
                        showObject=False,
                        showObjectScale=0.05,
                        showIndices=False, showIndicesScale=0.1)

        slidingpoints = rigid.addChild('SlidingPoints')
        slidingpoints.addObject('MechanicalObject', template='Rigid3', showObject=False, showObjectScale=0.01,
                                position=[
                                    [-r, -0.1, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                    [-r, 0, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                    [r * cos(3 * pi / 4), r * sin(3 * pi / 4), 0, 0, 0, sin(pi / 8), cos(pi / 8)],
                                    [0, r, 0, 0, 0, 0, 1],
                                    [r * cos(pi / 4), r * sin(pi / 4), 0, 0, 0, -sin(pi / 8), cos(pi / 8)]],
                                translation=[r, 0, 0]
                                )
        slidingpoints.addObject('RigidMapping', index=1, globalToLocalCoords=False)

        rigid.addObject('ArticulatedSystemMapping',
                        indexInput2=indexInput2,
                        input1=pulley.MechanicalObject.getLinkPath(),
                        input2=node.MechanicalObject.getLinkPath(),
                        output=rigid.MechanicalObject.getLinkPath())

        centers = rigid.addChild('ArticulationCenters')
        addArticulationCenter(centers, 'CenterY', 0, 1, [0, 0, 0], [0, 0., 0], 0, 0, 1, [0, 1, 0], 0)
        addArticulationCenter(centers, 'CenterZ', 1, 2, [r, 0, 0], [r, 0, 0], 0, 0, 1, [0, 0, 1], 1)

        for i in range(3):
            visual = rigid.addChild('Visual' + str(i))
            visual.addObject('MechanicalObject', template='Rigid3', showObject=False)
            visual.addObject('RigidMapping', index=i)

            ogl = visual.addChild('OGL')
            ogl.addObject('MeshSTLLoader', filename=path + 'mesh/pulley' + str(i + 1) + '.stl',
                          scale=0.001,
                          translation=[0.02, 0, 0]
                          )
            ogl.addObject('GenerateRigidMass', src='@MeshSTLLoader', density=1e3)
            ogl.addObject('OglModel', src='@MeshSTLLoader',
                          color=[[0.4, 0.4, 0.4, 1.],
                                 [0.4, 0.4, 0.4, 1.],
                                 [1, 1, 1, 1.]][i])
            ogl.addObject('RigidMapping', index=0)

            visual.addObject('UniformMass', vertexMass=ogl.GenerateRigidMass.rigidMass.linkpath)

        pulley.addObject(PulleyController(pulley, None))

        return pulley


def createScene(rootnode):
    from splib3.animation import animate, AnimationManager
    from scripts.utils.header import addHeader, addSolvers
    from scripts.cable import Cable
    from gui import CablesGUI
    from params import Parameters

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, rayleighStiffness=0.2, firstOrder=False)
    rootnode.addObject(AnimationManager(rootnode))
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields"
    rootnode.addObject('VisualGrid', plane='z', size=4, nbSubdiv=40, enable=False)
    rootnode.addObject('VisualGrid', plane='z', size=4, nbSubdiv=4, thickness=2, enable=False)
    rootnode.gravity.value = [0, -9.810, 0]

    ONLYPULLEY = False
    CABLEMODEL = "cosserat"  # "cosserat" or "beam"

    params = Parameters()
    params.cable.length = 2

    # Important parameters
    params.cable.youngModulus = 1.205e11  # in kg/mm/s2 -> N*1e3
    loadMass = 1000  # in kg

    coef = 2
    params.cable.nbSections = 40 * coef
    nbPointsOnPulley = 5 * coef

    length = params.cable.length
    nbSections = params.cable.nbSections
    r = params.pulley.radius

    pulleys = simulation.addChild('Pulleys')
    pulleys.addObject("MechanicalObject", position=[-r, (length - pi * r) / 2, 0, 0, 0, 0, 1], template='Rigid3')
    pulleys.addObject("FixedConstraint", indices=[0])

    pulley = Pulley(pulleys, indexInput2=0).node
    pulley.Rigid.Visual2.addObject("VisualStyle", displayFlags='showWireframe')
    slidingpoints = pulley.Rigid.SlidingPoints

    if not ONLYPULLEY:

        # Load
        load = simulation.addChild('Load')
        load.addObject('MechanicalObject', position=[r, 0, 0, 0, 0, 0, 1], template='Rigid3',
                       showObject=False, showObjectScale=0.00005)
        load.addObject('UniformMass', totalMass=loadMass, showAxisSizeFactor=0.1)
        visu = load.addChild('Visu')
        visu.addObject('MeshOBJLoader', filename='mesh/cube.obj')
        visu.addObject('OglModel', src='@MeshOBJLoader',
                       scale3d=[0.1, 0.1, 0.1], translation=[0, -0.1, 0])
        visu.addObject('RigidMapping')

        # Cable
        nbSectionsHalf = floor((nbSections - nbPointsOnPulley) / 2)
        dx = (length - pi * r) / 2 / nbSectionsHalf
        positions = [[- r, dx * i, 0, 0., 0., cos(pi/4), sin(pi/4)] for i in range(nbSectionsHalf + 1)]
        for i in range(nbPointsOnPulley):
            angle = - pi / 2 + (i + 1) * pi / nbPointsOnPulley
            positions += [[r * sin(angle), (length - pi * r) / 2 + r * cos(angle), 0., 0., 0., cos(angle/2 + pi/2), sin(angle/2 + pi/2)]]
        positions += [[r, (length - pi * r) / 2 - dx * (i + 1), 0, 0., 0., cos(-pi/4), sin(-pi/4)] for i in range(nbSections - nbSectionsHalf - nbPointsOnPulley)]

        if CABLEMODEL == "cosserat":
            for i, pos in enumerate(positions):
                pos[0:3] = [-r + i * dx, 0., 0.]

        cables = simulation.addChild('Cables')
        cable = Cable(modelling, cables,
                      positions=positions, length=length,
                      attachNode=load, attachIndex=0,
                      cableModel=CABLEMODEL, name="Cable").beam
        cable.base.addObject('FixedConstraint', indices=[0])

        # Cable pulley interaction
        difference = cable.rod.addChild('Difference')
        slidingpoints.addChild(difference)

        nbSlidingPoints = 5
        difference.addObject('MechanicalObject', template='Rigid3', position=[0, 0, 0, 0, 0, 0, 0] * nbSlidingPoints)
        difference.addObject('RestShapeSpringsForceField', points=list(range(nbSlidingPoints)), stiffness=1e12)
        difference.addObject('BeamProjectionDifferenceMultiMapping', template='Rigid3,Rigid3,Rigid3',
                             directions=[0, 1, 1, 0, 0, 0, 0],
                             indicesInput1=list(range(nbSlidingPoints)),
                             input1=slidingpoints.getMechanicalState().linkpath,
                             input2=cable.rod.getMechanicalState().linkpath,
                             interpolationInput2=cable.rod.BeamInterpolation.linkpath,
                             output=difference.getMechanicalState().linkpath,
                             draw=False, drawSize=0.01)
        if CABLEMODEL == "beam":
            pulley.PulleyController.cable = cable.node
            rootnode.addObject(CablesGUI(cables=cables))

    else:  # Animate pulley

        def animation(factor, target, value, index, direction, startTime=0):
            if factor > 0:
                pos = np.copy(target.position.value)
                pos[index][direction] = cos(value * factor)
                target.position.value = pos

        # Linear actuation
        targetObject = pulley.getMechanicalState()
        animate(animation, {'target': targetObject, 'value': pi, 'index': 0, 'direction': 0, 'startTime': 0},
                duration=2, mode='pingpong')
        animate(animation, {'target': targetObject, 'value': 2 * pi, 'index': 1, 'direction': 0, 'startTime': 0},
                duration=2, mode='pingpong')
