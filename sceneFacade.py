from scripts.system import System
from params import Parameters


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    params = Parameters()
    params.structure.height = 9.5
    params.structure.width = 1
    params.structure.length = 27.78
    params.structure.thickness = 0

    defaultNbSections = 80
    params.cable.nbSections = 3 * defaultNbSections

    defaultLength = 5
    params.cable.length = 3 * defaultLength

    params.structure.pulleysorientations = [0, 0, 0, 0,
                                            0, 0, 0, 0]

    params.structure.pulleysUD = ["up", "down", "up", "down",
                                  "up", "down", "up", "down"]

    params.structure.cornersOrder = [2, 3, 1, 0, 6, 7, 5, 4]

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False, rayleighStiffness=0.2)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields"

    system = System(modelling, simulation, cableModel='beam')
    for i, cable in enumerate(system.cables.children):
        cable.RigidBase.addObject('RestShapeSpringsForceField', points=[0], stiffness=1e12)

