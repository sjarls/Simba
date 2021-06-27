from Machines import Cylinder, Winch

# Simulation 
endTime = 3.0
startTime = 0
dt = 0.0001
ad_max = 50/10e6

# Instanciate machines
'''
Cylinder args:  bore radius A, rod radius A, bore radius B, rod radius B,
                initial position, initial speed, payload mass
Winch args:     drum inertia, drum radius, friction, gear ratio, motor inertia,
                motor displacement, payload mass
'''
Cyl1 = Cylinder(0.100/2, 0, 0.100/2, 0.085/2, 0.5, 0, 10000)
Winch1 = Winch(300, 0.6, 10, 34.5, 0.2, 230e-6, 1000)
Winch2 = Winch(300, 0.4, 10, 34.5, 0.2, 230e-6, 430)


while Cyl1.t < endTime:
    '''
    Valve commands are either simulated locally or received
    from external source/HIL.
    '''
    if Cyl1.t < 0.5:
        ad = ad_max/2.0*Cyl1.t
    else:
        ad = ad_max
    Cyl1.moveCylinder(ad, 2.4e7)
    Winch1.moveWinch(ad, 2.9e7)
    Winch2.moveWinch(ad, 2.3e7)

'''
All params are plottet with time as x-axis
Available y-axis parameters: 
Flow:       qPlot_PA    qPlot_PB    qPlot_BT
Pressure:   pPlot_p     pPlot_A     pPlot_B
Pos/Vel:    yDotPlot    yDotPlot
Valve Area: adPlot
'''

Cyl1.plotData(Cyl1.pPlot_A, Cyl1.pPlot_B)
Winch1.plotData(Winch1.thetaDotPlot, Winch2.thetaDotPlot)
Winch2.plotData(Winch2.p_APlot, Winch2.p_BPlot)

'''
TO DO:
1. Make plotData a stand-alone function
   Create more informative plots with named legends etc.
2. Create a wave class to simulate heave motion
   Inputs: frequency, amplitude
   Output: heave position and velocity.
   (Pitch/roll/yaw...?)
3. Create a control class to implement simple PID controls
   Inputs: heave motion, payload motion feedback, reference
   Outputs: depending on control element: valve command, pressure setting, flow..
4. Inheritance..? The classes will have some similarity
5. ...
'''