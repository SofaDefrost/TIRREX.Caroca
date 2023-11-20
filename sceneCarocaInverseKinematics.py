from scripts.system import System


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    addSolvers(simulation, firstOrder=False)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    System(modelling, simulation, cableModel='beam', inverse=True)

    rootnode.addObject('VisualGrid', size=10, nbSubdiv=100)
