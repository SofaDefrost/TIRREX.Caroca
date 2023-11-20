from scripts.system import System


def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False, rayleighStiffness=0.2)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    caroca = System(modelling, simulation, cableModel='beam')
    for i, cable in enumerate(caroca.cables.children):
        cable.RigidBase.addObject('RestShapeSpringsForceField', points=[0], stiffness=1e12)
