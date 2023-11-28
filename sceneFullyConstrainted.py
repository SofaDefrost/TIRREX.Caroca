from scripts.system import System
from params import Parameters


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    params = Parameters()
    params.structure.height = 9.5
    params.structure.width = 18.45
    params.structure.length = 28.45
    params.structure.thickness = 0

    defaultNbSections = 80
    params.cable.nbSections = 4 * defaultNbSections

    defaultLength = 5
    params.cable.length = 4 * defaultLength

    params.structure.pulleysorientations = [0, 0, 0, 0,
                                            0, 0, 0, 0]

    params.structure.pulleysUD = ["up", "down", "up", "down",
                                  "up", "down", "up", "down"]

    params.structure.cornersOrder = [3, 1, 2, 0, 7, 4, 6, 4]

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False, rayleighStiffness=0.2)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    system = System(modelling, simulation, cableModel='beam')
    for i, cable in enumerate(system.cables.children):
        cable.RigidBase.addObject('RestShapeSpringsForceField', points=[0], stiffness=1e12)

