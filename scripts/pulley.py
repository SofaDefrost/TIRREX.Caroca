from scripts.utils import addArticulationCenter
import os
from math import sin, cos, pi, floor

path = os.path.dirname(os.path.realpath(__file__)) + '/../'


class Pulley:
    """
    Pulley object
    """

    def __init__(self, node, structure=None, position=[0, 0, 0], angle=0, indexInput2='', name="Pulley"):
        self.__addPulley(node, structure, position, angle, indexInput2, name)

    def __addPulley(self, node, structure, position, angle, indexInput2, name):
        self.node = node.addChild(name)
        pulley = self.node
        pulley.addObject("MechanicalObject", position=[angle, 0], template='Vec1')
        pulley.addObject('RestShapeSpringsForceField', name='rssff1', points=[0], stiffness=1e-4)
        pulley.addObject('RestShapeSpringsForceField', name='rssff2', points=[1], stiffness=1e12)
        pulley.addObject('ArticulatedHierarchyContainer')

        rigid = pulley.addChild('Rigid')
        rigid.addObject('MechanicalObject', template='Rigid3',
                        position=[[0, 0, 0, 0, 0, 0, 1]] * 3,
                        translation=[0, 0.09, 0],
                        showObject=False,
                        showObjectScale=0.1,
                        showIndices=False, showIndicesScale=0.1)

        slidingpoints = rigid.addChild('SlidingPoints')
        pulleyRadius = 0.075
        slidingpoints.addObject('MechanicalObject', template='Rigid3', showObject=True, showObjectScale=0.01,
                                position=[
                                    [pulleyRadius * cos(3 * pi / 4), pulleyRadius * sin(3 * pi / 4), 0, 0, 0, 0, 1],
                                    [0, pulleyRadius, 0, 0, 0, 0, 1],
                                    [pulleyRadius * cos(pi / 4), pulleyRadius * sin(pi / 4), 0, 0, 0, 0, 1]])
        slidingpoints.addObject('RigidMapping', index=2, globalToLocalCoords=False)

        if structure is not None:
            rigid.addObject('ArticulatedSystemMapping',
                            indexInput2=indexInput2,
                            input1=pulley.MechanicalObject.getLinkPath(),
                            input2=structure.MechanicalObject.getLinkPath(),
                            output=rigid.MechanicalObject.getLinkPath())
        else:
            rigid.addObject('ArticulatedSystemMapping',
                            input1=pulley.MechanicalObject.getLinkPath(),
                            output=rigid.MechanicalObject.getLinkPath())

        centers = rigid.addChild('ArticulationCenters')
        addArticulationCenter(centers, 'CenterY', 0, 1, position, [0, 0., 0], 0, 0, 1, [0, 1, 0], 0)
        addArticulationCenter(centers, 'CenterZ', 1, 2, [0, -0.165, 0], [0, 0, 0], 0, 0, 1, [0, 0, 1], 1)

        visual = rigid.addChild('Visual1')
        visual.addObject('MeshSTLLoader', filename=path + 'mesh/pulley1.stl', scale=0.02,
                         translation=[position[j] + [0, -0.265, 0.005][j] for j in range(3)],
                         rotation=[0, 0, 180])
        visual.addObject('OglModel', src='@MeshSTLLoader')
        visual.addObject('RigidMapping', src='@MeshSTLLoader', index=0)

        visual = rigid.addChild('Visual2')
        visual.addObject('MeshSTLLoader', filename=path + 'mesh/pulley2.stl', scale=0.02, translation=[0, -0.265, 0],
                         rotation=[0, 90, 180])
        visual.addObject('OglModel', src='@MeshSTLLoader')
        visual.addObject('RigidMapping', src='@MeshSTLLoader', index=1)

        visual = rigid.addChild('Visual3')
        visual.addObject('MeshSTLLoader', filename=path + 'mesh/pulley3.stl', scale=0.02, translation=[0, -0.085, 0],
                         rotation=[0, 90, 180])
        visual.addObject('OglModel', src='@MeshSTLLoader')
        visual.addObject('RigidMapping', src='@MeshSTLLoader', index=2)

        return pulley


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers
    from scripts.cable import Cable
    from gui import CablesGUI
    import params

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, rayleighStiffness=0.1, firstOrder=False)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"
    rootnode.addObject('VisualGrid', plane='z', size=4, nbSubdiv=40)
    rootnode.addObject('VisualGrid', plane='z', size=4, nbSubdiv=4, thickness=2)

    length = 2
    nbSections = params.CableParameters.nbSections
    dx = length / nbSections

    load = simulation.addChild('Load')
    load.addObject('MechanicalObject', position=[dx * 2, 0, 0, 0, 0, 0, 1], template='Rigid3',
                   showObject=True, showObjectScale=0.05)
    load.addObject('UniformMass', totalMass=400)
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

    slidingpoints = Pulley(simulation, position=[0, length / 2, 0]).node.Rigid.SlidingPoints

    difference = cable.rod.addChild('Difference')
    slidingpoints.addChild(difference)

    difference.addObject('MechanicalObject', template='Rigid3', position=[0, 0, 0, 0, 0, 0, 0] * 3)
    difference.addObject('RestShapeSpringsForceField', points=list(range(3)), stiffness=1e12)
    difference.addObject('BeamProjectionDifferenceMultiMapping', template='Rigid3,Rigid3,Rigid3',
                         directions=[0, 1, 1, 0, 0, 0, 0],
                         indicesInput1=list(range(3)),
                         input1=slidingpoints.getMechanicalState().linkpath,
                         input2=cable.rod.getMechanicalState().linkpath,
                         interpolationInput2=cable.rod.BeamInterpolation.linkpath,
                         output=difference.getMechanicalState().linkpath,
                         draw=False, drawSize=0.1)

    rootnode.addObject(CablesGUI(cables=cables))
