import Sofa.Core
from splib3.numerics import Quat, Vec3
from math import pi


class BaseObject:

    node: Sofa.Core.Node
    base: Sofa.Core.Node
    deformable: Sofa.Core.Node
    rod: Sofa.Core.Node

    def __init__(self, modelling, simulation, params, length, name='BaseBeam', positionBase=[0, 0, 0, 0, 0, 0, 1],
                 collisionGroup=0):
        self.modelling = modelling
        self.simulation = simulation
        self.params = params
        self.name = name
        self.collisionGroup = collisionGroup
        self.length = length

        self.positionBase = positionBase
        self.translation = positionBase[0:3]
        self.rotation = Quat(positionBase[3:7]).getEulerAngles()
        self.rotation = [r * 180 / pi for r in self.rotation]

    def addCylinderTopology(self):
        positions = self.rod.MechanicalObject.position.value

        topology = self.modelling.Topology.addChild(self.name + 'CylinderTopo')
        edgetopo = topology.addChild('Edge')
        edgetopo.addObject('EdgeSetTopologyContainer', edges=[[k, k + 1] for k in range(len(positions) - 1)])
        edgetopo.addObject('EdgeSetTopologyModifier')
        edgetopo.addObject('MechanicalObject', template='Rigid3',
                           position=self.rod.MechanicalObject.position.getLinkPath())

        quadtopo = edgetopo.addChild('Quad')
        quadtopo.addObject('QuadSetTopologyContainer')
        quadtopo.addObject('QuadSetTopologyModifier')
        quadtopo.addObject('MechanicalObject')
        quadtopo.addObject('Edge2QuadTopologicalMapping',
                           input=edgetopo.EdgeSetTopologyContainer.getLinkPath(),
                           output=quadtopo.QuadSetTopologyContainer.getLinkPath(),
                           flipNormals=True, nbPointsOnEachCircle=10, radius=self.params.radius)

    def addVisualModel(self):
        quadtopo = self.modelling.Topology.getChild(self.name + 'CylinderTopo').Edge.Quad

        visual = self.rod.addChild('VisualModel')
        visual.addObject('MeshTopology', position=quadtopo.MechanicalObject.position.getLinkPath(),
                         quads=quadtopo.QuadSetTopologyContainer.quads.getLinkPath())
        visual.addObject('OglModel', color=[0.1, 0.1, 0.1, 1.0])
        visual.addObject('SkinningMapping')

    def addCollisionModel(self):
        quadtopo = self.modelling.Topology.getChild(self.name + 'CylinderTopo').Edge.Quad

        collision = self.rod.addChild('CollisionModel')
        collision.addObject('MeshTopology', position=quadtopo.MechanicalObject.position.getLinkPath(),
                            quads=quadtopo.QuadSetTopologyContainer.quads.getLinkPath())
        collision.addObject('MechanicalObject')
        collision.addObject('TriangleCollisionModel', group=self.collisionGroup)
        collision.addObject('PointCollisionModel', group=self.collisionGroup)
        collision.addObject('LineCollisionModel', group=self.collisionGroup)
        collision.addObject('SkinningMapping')

