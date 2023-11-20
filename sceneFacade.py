from scripts.system import System
from params import Parameters


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    params = Parameters()
    params.structure.height = 9.5
    params.structure.width = 1
    params.structure.length = 27.78
    params.structure.thickness = 0

    params.cable.nbSections *= 3
    params.cable.length *= 3

    params.structure.pulleysorientations = [0, 0, 0, 0,
                                            0, 0, 0, 0]

    params.structure.pulleysUD = ["down", "up", "up", "down",
                                  "up", "down", "down", "up"]

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False, rayleighStiffness=0.2)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    system = System(modelling, simulation, cableModel='beam')
    for i, cable in enumerate(system.cables.children):
        cable.RigidBase.addObject('RestShapeSpringsForceField', points=[0], stiffness=1e12)

