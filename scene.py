from scripts.caroca import Caroca


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    Caroca(modelling, simulation, cableModel='beam')

    rootnode.addObject('VisualGrid', size=10, nbSubdiv=100)
