import Sofa
import Sofa.Core

import tkinter as tkinter
import threading
from math import pi, floor
from params import Parameters


class App(threading.Thread):

    def __init__(self, cables):
        threading.Thread.__init__(self)
        self.daemon = True
        self.cables = cables
        self.start()

    def callback(self):
        self.root.quit()

    def addScale(self, frame, row, column, resolution, from_, to, var):
        scale = tkinter.Scale(frame, width=floor(self.root.winfo_screenwidth() * 0.005),
                              variable=var, resolution=resolution,
                              from_=from_, to=to, showvalue=0,
                              orient=tkinter.HORIZONTAL)
        scale.grid(row=row, column=column,
                   padx=self.padx, pady=self.pady, sticky="NSEW")
        tkinter.Grid.columnconfigure(frame, column, weight=1)

    def run(self):
        self.root = tkinter.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)
        self.root.title("Cables Command")
        self.root.minsize(300, 300)
        self.font = "SegoeUISymbol 10"
        self.tittlefont = "SegoeUISymbol 10 bold"
        self.padx = floor(self.root.winfo_screenwidth() * 0.003)
        self.pady = floor(self.root.winfo_screenwidth() * 0.003)

        tkinter.Grid.columnconfigure(self.root, 0, weight=1)
        tkinter.Grid.columnconfigure(self.root, 1, weight=1)

        # Sliders
        cablesFrame = tkinter.LabelFrame(self.root, text=" Cables ", font=self.tittlefont)
        cablesFrame.grid(row=0, columnspan=2, padx=self.padx, pady=self.pady, sticky="NSEW")
        cablesFrame.pack(fill="both", expand=True)
        tkinter.Grid.columnconfigure(cablesFrame, 0, weight=1)

        self.sliders = []
        for i, cable in enumerate(self.cables.children):
            slider = tkinter.DoubleVar()
            self.sliders.append(slider)
            tkinter.Grid.columnconfigure(self.root, i, weight=1)
            tkinter.Label(cablesFrame, text="Cable" + str(i),
                          font=self.font).grid(row=i + 1, column=0, padx=self.padx, pady=self.pady, sticky="NSEW")
            self.addScale(cablesFrame, i + 1, 1, 0.001, 0, -cable.length.value / 2, slider)

        self.root.mainloop()


class CablesGUI(Sofa.Core.Controller):
    params = Parameters()

    def __init__(self, cables):

        Sofa.Core.Controller.__init__(self, cables)
        self.name = "CablesGUI"
        self.cables = cables

        self.app = App(cables=self.cables)

        return

    def onAnimateBeginEvent(self, event):

        # Update the simulation according to the GUI
        if self.cables is not None:

            for i, cable in enumerate(self.cables.children):
                displacement = cable.BeamController.displacement
                if displacement < self.app.sliders[i].get():
                    cable.velocity.value = self.params.velocity
                elif displacement > self.app.sliders[i].get():
                    cable.velocity.value = -self.params.velocity
                cable.displacement.value = self.app.sliders[i].get()

        return


# Test/example scene
def createScene(rootnode):
    from scripts.cable import Cable
    from scripts.utils.header import addHeader, addSolvers
    import params

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation, firstOrder=False)
    rootnode.VisualStyle.displayFlags = "showInteractionForceFields showCollisionModels"

    cables = simulation.addChild('Cables')

    length = 2

    load = simulation.addChild('Load')
    load.addObject('MechanicalObject', position=[[length, 0, 0, 0, 0, 0, 1]], template='Rigid3',
                   showObject=True, showObjectScale=0.1)
    load.addObject('UniformMass', totalMass=10)

    nbSections = params.CableParameters.nbSections
    dx = length / nbSections
    positions = [[dx * i, 0, 0, 0, 0, 0, 1] for i in range(nbSections + 1)]
    cable = Cable(modelling, cables,
                  positions=positions, length=length,
                  attachNode=load, attachIndex=0,
                  cableModel="beam", name="Cable0").beam
    cable.base.addObject('FixedConstraint', indices=[0])

    rootnode.addObject(CablesGUI(cables=cables))

    return
