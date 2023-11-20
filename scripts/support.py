import os
path = os.path.dirname(os.path.realpath(__file__)) + '/../'


class Support:
    """
    Support system to attach / deploy / retract the cables.
    Visual model only.
    """

    def __init__(self, modelling,
                 translation=[0, 0, 0], rotation=[0, 0, 0],
                 color=[1, 1, 1, 1], filename='support.stl', name="Support"):
        self.modelling = modelling
        self.translation = translation
        self.rotation = rotation
        self.color = color
        self.filename = filename
        self.name = name
        self.__addSupport()

    def __addSupport(self):
        self.support = self.modelling.addChild(self.name)
        support = self.support

        support.addObject('MeshSTLLoader', filename=path + 'mesh/' + self.filename)
        support.addObject('OglModel', src="@MeshSTLLoader",
                          color=self.color, translation=self.translation, rotation=self.rotation)


def createScene(rootnode):
    from scripts.utils.header import addHeader
    from scripts.pulley import Pulley

    settings, modelling, simulation = addHeader(rootnode)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"
    rootnode.addObject('VisualGrid', plane='z', size=4, nbSubdiv=40)
    rootnode.addObject('VisualGrid', plane='z', size=4, nbSubdiv=4, thickness=2)
    rootnode.gravity.value = [0, -9810, 0]

    Support(modelling, translation=[0, 0, 0])

    pulleys = simulation.addChild('Pulleys')
    pulleys.addObject("MechanicalObject", position=[0, 0, 0, 0, 0, 0, 0, 1], template='Rigid3')
    pulleys.addObject("FixedConstraint", indices=[0])
    Pulley(pulleys, indexInput2=0).node

