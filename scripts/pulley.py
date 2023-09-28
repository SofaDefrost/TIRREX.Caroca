from scripts.utils import addArticulationCenter
import os
path = os.getcwd() + '/../'


class Pulley:
    """
    Pulley object
    """

    def __init__(self, node, structure=None, position=[0, 0, 0], angle=0, indexInput2='', name="Pulley"):
        self.node = self.__addPulley(node, structure, position, angle, indexInput2, name)

    def __addPulley(self, node, structure, position, angle, indexInput2, name):
        self.node = node.addChild(name)
        pulley = self.node
        pulley.addObject("MechanicalObject", position=[angle, 0], template='Vec1')
        pulley.addObject('RestShapeSpringsForceField', points=[0, 1], stiffness=1, angularStiffness=1)
        pulley.addObject('ArticulatedHierarchyContainer')

        rigid = pulley.addChild('Rigid')
        rigid.addObject('MechanicalObject', template='Rigid3',
                        position=[position + [0, 0, 0, 1]] * 3, showObject=True,
                        showObjectScale=0.1,
                        showIndices=False, showIndicesScale=0.1)
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
        visual.addObject('MeshSTLLoader', filename=path+'mesh/pulley1.stl', scale=0.02,
                         translation=[position[j] + [0, -0.265, 0.005][j] for j in range(3)],
                         rotation=[0, 0, 180])
        visual.addObject('OglModel', src='@MeshSTLLoader')
        visual.addObject('RigidMapping', src='@MeshSTLLoader', index=0)

        visual = rigid.addChild('Visual2')
        visual.addObject('MeshSTLLoader', filename=path+'mesh/pulley2.stl', scale=0.02, translation=[0, -0.265, 0],
                         rotation=[0, 90, 180])
        visual.addObject('OglModel', src='@MeshSTLLoader')
        visual.addObject('RigidMapping', src='@MeshSTLLoader', index=1)

        visual = rigid.addChild('Visual3')
        visual.addObject('MeshSTLLoader', filename=path+'mesh/pulley3.stl', scale=0.02, translation=[0, -0.085, 0],
                         rotation=[0, 90, 180])
        visual.addObject('OglModel', src='@MeshSTLLoader')
        visual.addObject('RigidMapping', src='@MeshSTLLoader', index=2)

        return pulley


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    Pulley(simulation)
