from scripts.caroca import Caroca


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False, rayleighStiffness=0.1)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    caroca = Caroca(modelling, simulation, cableModel='beam')
    for i, cable in enumerate(caroca.cables.children):
        cable.RigidBase.addObject('RestShapeSpringsForceField', points=[0], stiffness=1e12)

    rootnode.addObject('VisualGrid', size=10, nbSubdiv=100)
