from scripts.utils.baseobject import BaseObject
from scripts.utils.cosserat.utils import getStrainFromAngles
from splib3.numerics import Quat, Vec3, vsub
from math import pi
import numpy as np


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

        indexPairs = [[0, 0]]
        for i in range(nbSections):
            indexPairs += [[1, i]]

        self.base = self.node.addChild('RigidBase')
        self.base.addObject('MechanicalObject', template='Rigid3', position=self.positions[0])

        rod = self.base.addChild('Rod')
        self.rod = rod
        self.deformable = self.node.addChild('Deformable')
        self.deformable.addChild(rod)

        rod.addObject('EdgeSetTopologyContainer',
                      position=[pos[0:3] for pos in self.positions],
                      edges=[[i, i+1] for i in range(nbSections)])
        rod.addObject('MechanicalObject', template='Rigid3',
                      position=self.positions, showIndices=False, showIndicesScale=0.005)
        rod.addObject('BeamInterpolation')
        rod.init()
        lengths = [l for l in rod.BeamInterpolation.lengthList.value]

        # Convert Rigid3 orientation description to Cosserat bending description
        # [[torsion strain, y_bending strain, z_bending strain]]
        strain = []
        for i in range(len(self.positions) - 1):
            q1 = Quat(self.positions[i][3:7])
            q2 = Quat(self.positions[i + 1][3:7])
            q = q2.rotateFromQuat(q1.getInverse())
            angles = q.getEulerAngles()
            # lengths[i] = Vec3(vsub(self.positions[i][0:3], self.positions[i + 1][0:3])).getNorm()
            strain.append(getStrainFromAngles(angles, lengths[i]))

        self.deformable.addObject('MechanicalObject', position=strain, rest_position=[0, 0, 0] * nbSections)
        self.deformable.addObject('BeamHookeLawForceField',
                                  youngModulus=self.params.youngModulus,
                                  poissonRatio=self.params.poissonRatio,
                                  radius=self.params.radius,
                                  length=lengths)
        self.node.addData(name="indexExtremity", type='int', value=nbPoints - 1)

        totalMass = self.params.density * self.length * self.params.radius * self.params.radius * pi
        rod.addObject('UniformMass', totalMass=totalMass)
        rod.addObject('DiscreteCosseratMapping',
                      curv_abs_input=[self.positions[i][0] for i in range(nbPoints)],
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
