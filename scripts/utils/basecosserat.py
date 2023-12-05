from scripts.utils.baseobject import BaseObject
from splib3.numerics import Quat
from math import pi


class BaseCosserat(BaseObject):
    """
    Base rod, based on Cosserat. With a visual and a collision model.
    """

    deformabletemplate = 'Vec3'

    def __init__(self, modelling, simulation, params, positions, length, name='BaseCosserat', collisionGroup=0):
        super().__init__(modelling, simulation, params, positions, length, name, collisionGroup)
        self.__addRod()
        self.addCylinderTopology()
        self.addVisualModel()

    def __addRod(self):
        self.node = self.modelling.addChild(self.name)
        self.simulation.addChild(self.node)
        self.node.addObject('RequiredPlugin', pluginName=['Cosserat'])

        nbSections = self.params.nbSections
        nbPoints = nbSections + 1
        dx = self.length / nbSections

        indexPairs = [[0, 0]]
        for i in range(nbSections):
            indexPairs += [[1, i]]

        self.base = self.node.addChild('RigidBase')
        self.base.addObject('MechanicalObject', template='Rigid3', position=self.positions[0])

        self.deformable = self.node.addChild('Deformable')

        # Convert Rigid3 orientation description to Cosserat bending description
        # [[torsion strain, y_bending strain, z_bending strain]]
        strain = []
        for i in range(len(self.positions) - 1):
            q1 = Quat(self.positions[i][3:7])
            q2 = Quat(self.positions[i + 1][3:7])
            q = q1.rotateFromQuat(q2.getInverse())
            angles = q.getEulerAngles()  # TODO: angles to strain
            strain.append([0., 0., 0.])

        self.deformable.addObject('MechanicalObject', position=strain)
        self.deformable.addObject('BeamHookeLawForceField',
                                  youngModulus=self.params.youngModulus,
                                  poissonRatio=self.params.poissonRatio,
                                  radius=self.params.radius,
                                  length=[dx] * nbSections)
        self.node.addData(name="indexExtremity", type='int', value=nbPoints - 1)

        rod = self.base.addChild('Rod')
        self.rod = rod
        self.deformable.addChild(rod)

        nbSections = self.params.nbSections
        nbPoints = nbSections + 1
        dx = self.length / nbSections

        rod.addObject('EdgeSetTopologyContainer',
                      position=[pos[0:3] for pos in self.positions],
                      edges=[[i, i+1] for i in range(nbSections)])
        rod.addObject('MechanicalObject', template='Rigid3',
                      position=[pos[0:3] + [0., 0., 0., 1.] for pos in self.positions]
                      )
        rod.addObject('BeamInterpolation')

        totalMass = self.params.density * self.length * self.params.radius * self.params.radius * pi
        inertiaMatrix = [[1 / 2 * self.params.radius*self.params.radius, 0, 0],
                         [0, 1 / 12 * (3 * self.params.radius * self.params.radius + dx * dx), 0],
                         [0, 0, 1 / 12 * (3 * self.params.radius * self.params.radius + dx * dx)]]
        vertexMass = [totalMass/nbPoints, 1, inertiaMatrix]
        rod.addObject('UniformMass', totalMass=totalMass)
        rod.addObject('DiscreteCosseratMapping',
                      curv_abs_input=[self.positions[i][0] for i in range(0, nbPoints, 2)],
                      curv_abs_output=[self.positions[i][0] for i in range(nbPoints)],
                      input1=self.deformable.MechanicalObject.getLinkPath(),
                      input2=self.base.MechanicalObject.getLinkPath(),
                      output=rod.getLinkPath(),
                      debug=False, baseIndex=0)

        return rod


# Test scene
def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers
    import params

    settings, modelling, simulation = addHeader(rootnode)
    rootnode.VisualStyle.displayFlags = ['hideBehavior']
    addSolvers(simulation, iterative=False)

    nbSections = params.CableParameters.nbSections
    length = 5
    dx = length / nbSections
    positions = [[dx * i, 0, 0, 0, 0, 0, 1] for i in range(nbSections + 1)]
    beam = BaseCosserat(modelling, simulation, params.CableParameters, positions, length)
    beam.node.RigidBase.addObject('FixedConstraint', indices=0)
