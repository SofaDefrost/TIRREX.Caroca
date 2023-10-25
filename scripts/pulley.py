from scripts.utils import addArticulationCenter
import os
from math import sin, cos, pi, floor
from params import Parameters

path = os.path.dirname(os.path.realpath(__file__)) + '/../'


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
                        showObjectScale=0.1,
                        showIndices=False, showIndicesScale=0.1)

        slidingpoints = rigid.addChild('SlidingPoints')
        slidingpoints.addObject('MechanicalObject', template='Rigid3', showObject=True, showObjectScale=0.01,
                                position=[
                                    [-r, -0.1, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                    [-r, 0, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                    [r * cos(3 * pi / 4), r * sin(3 * pi / 4), 0, 0, 0,
                                     sin(pi / 8), cos(pi / 8)],
                                    [0, r, 0, 0, 0, 0, 1],
                                    [r * cos(pi / 4), r * sin(pi / 4), 0, 0, 0, -sin(pi / 8),
                                     cos(pi / 8)]],
                                translation=[r - 0.02, 0, 0]
                                )
        slidingpoints.addObject('RigidMapping', index=2, globalToLocalCoords=False)

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
                          scale=0.001
                          )
            ogl.addObject('GenerateRigidMass', src='@MeshSTLLoader', density=1e3)
            ogl.addObject('OglModel', src='@MeshSTLLoader')
            ogl.addObject('RigidMapping', index=0)

            visual.addObject('UniformMass', vertexMass=ogl.GenerateRigidMass.rigidMass.linkpath)

        return pulley


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers
    from scripts.cable import Cable
    from gui import CablesGUI
    import params

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, rayleighStiffness=0.2, firstOrder=False)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"
    rootnode.addObject('VisualGrid', plane='z', size=4, nbSubdiv=40)
    rootnode.addObject('VisualGrid', plane='z', size=4, nbSubdiv=4, thickness=2)
    rootnode.gravity.value = [0, -9810, 0]

    onlyPulley = False

    length = 2
    nbSections = params.CableParameters.nbSections
    dx = length / nbSections

    pulleys = simulation.addChild('Pulleys')
    pulleys.addObject("MechanicalObject", position=[-0.03, length / 2 - 0.075, 0, 0, 0, 0, 1], template='Rigid3')
    pulleys.addObject("FixedConstraint", indices=[0])

    slidingpoints = Pulley(pulleys, indexInput2=0).node.Rigid.SlidingPoints

    if not onlyPulley:
        load = simulation.addChild('Load')
        load.addObject('MechanicalObject', position=[dx * 2, 0, 0, 0, 0, 0, 1], template='Rigid3',
                       showObject=False, showObjectScale=0.05)
        load.addObject('UniformMass', totalMass=1)
        visu = load.addChild('Visu')
        visu.addObject('MeshOBJLoader', filename='mesh/cube.obj')
        visu.addObject('OglModel', src='@MeshOBJLoader',
                       scale3d=[0.1, 0.1, 0.1], translation=[0, -0.1, 0])
        visu.addObject('RigidMapping')

        positions = [[- dx * 2, dx * i, 0, 0, 0, 0.707, 0.707] for i in range(floor(nbSections / 2))]
        positions += [[0, length / 2, 0, 0, 0, 0, 1]]
        positions += [[dx * 2, length / 2 - dx * (i + 1), 0, 0, 0, -0.707, 0.707] for i in range(floor(nbSections / 2))]

        cables = simulation.addChild('Cables')
        cable = Cable(modelling, cables,
                      positions=positions, length=length,
                      attachNode=load, attachIndex=0,
                      cableModel='beam', name="Cable").beam
        cable.base.addObject('FixedConstraint', indices=[0])

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
                             draw=False, drawSize=0.1)

        rootnode.addObject(CablesGUI(cables=cables))
