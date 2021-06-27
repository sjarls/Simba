import matplotlib.pyplot as plt

pi = 3.14     # pi...
g = 9.82      # gravitational constant
beta_o = 1e9  # fluid stiffness
rho_o = 850   # fluid density 
cd = 0.6      # valve discharge coefficient
dt = 1e-4     # time step

plt.style.use('default')
fig, ax = plt.subplots()

class Cylinder():
  def __init__(self, r1_cyl, r1_st, r2_cyl, r2_st, y_0, yDot_0, m_pl):
    # Cylinder dimensions
    self.r1_cyl = r1_cyl
    self.r1_st  = r1_st
    self.r2_cyl = r2_cyl
    self.r2_st  = r2_st
    self.a1_cyl = pi*self.r1_cyl**2 - pi*self.r1_st**2
    self.a2_cyl = pi*self.r2_cyl**2 - pi*self.r2_st**2
    self.m_pl   = m_pl

    # Cylinder initial conditions
    self.y_0    = y_0
    self.y      = y_0
    self.yDot   = yDot_0
    self.p_A    = m_pl*g/self.a1_cyl
    self.p_B    = 0
    self.pDot_A = 0
    self.pDot_B = 0
    self.q_PA   = 0
    self.q_BT   = 0
    self.q_PB   = 0
    self.q_AT   = 0

    # Simulation properties
    self.pPlot_p  = []
    self.qPlot_PA = []
    self.qPlot_PB = []
    self.qPlot_BT = []
    self.pPlot_p  = []
    self.pPlot_A  = []
    self.pPlot_B  = []
    self.tPlot    = []
    self.yPlot    = []
    self.yDotPlot = []
    self.adPlot   = []

    self.t  = 0

  def moveCylinder(self, ad, p_p):
    # Flow equations and pressure differentials accross directional valve
    if ad > 0:
      self.q_PA = cd*ad*(2/rho_o*abs(p_p-self.p_A))**0.5
      self.q_BT = cd*ad*(2/rho_o*abs(self.p_B))**0.5

      self.pDot_A = beta_o*(self.q_PA - self.yDot*self.a1_cyl)/(0.001 + (self.y_0 + self.y)*self.a1_cyl)
      self.pDot_B = beta_o*(self.yDot*self.a2_cyl - self.q_BT)/(0.001 + (self.y_0 - self.y)*self.a2_cyl)

    elif ad < 0:
      self.q_PB = cd*ad*(2/rho_o*abs(p_p-self.p_B))**0.5
      self.q_AT = cd*ad*(2/rho_o*abs(self.p_A))**0.5  

      self.pDot_A = beta_o*(-self.q_AT - self.yDot*self.a1_cyl)/((self.y_0 + self.y)*self.a1_cyl)
      self.pDot_B = beta_o*(self.yDot*self.a2_cyl + self.q_PB)/((self.y_0 - self.y)*self.a2_cyl)
    
    else:
      self.q_PA = 0
      self.q_BT = 0
      self.pDot_A = 0
      self.pDot_B = 0

      self.q_PB = 0
      self.q_AT = 0
      self.pDot_A = 0
      self.pDot_B = 0
    
    self.updateState(ad, p_p)

  # Forward Euler and differential equation
  def updateState(self, ad, p_p):
    self.f_cyl = self.p_A*self.a1_cyl - self.p_B*self.a2_cyl
    self.yDotDot = (self.f_cyl - self.m_pl*g)/self.m_pl

    self.t = self.t + dt
    self.y = self.y + self.yDot*dt + self.yDotDot*dt
    self.yDot = self.yDot + self.yDotDot*dt

    self.p_A = self.p_A + self.pDot_A*dt
    self.p_B = self.p_B + self.pDot_B*dt

    self.t = self.t + dt

    self.tPlot.append(self.t)
    self.yPlot.append(self.y)
    self.yDotPlot.append(self.yDot)
    self.pPlot_p.append(p_p/1e5) 
    self.adPlot.append(ad)
    self.pPlot_A.append(self.p_A/1e5) 
    self.pPlot_B.append(self.p_B/1e5)
    self.qPlot_PA.append(self.q_PA*6e4)
    self.qPlot_PB.append(self.q_PB*6e4)
    self.qPlot_BT.append(self.q_BT*6e4)

  def plotData(self, *args):
    index = 0
    for arg in args:
      plt.plot(self.tPlot, arg, linewidth = 1, label = index)
      plt.legend()
      index += 1
    plt.draw()
    plt.show()

class Winch():
  def __init__(self, J_dr, r_dr, fric, i_mot, J_m, D_m, m_pl):
    self.J_dr  = J_dr
    self.r_dr  = r_dr
    self.fric  = fric
    self.i_mot = i_mot
    self.J_m   = J_m
    self.D_m   = D_m
    self.m_pl  = m_pl
    self.Jeff  = J_m + J_dr/i_mot**2 + m_pl*r_dr/i_mot**2
    self.Meff  = 0
    self.Va    = 1e-3
    self.Vb    = 1e-3
    self.p_B   = 100e5
    self.p_A   = 130e5

    self.theta        = 0
    self.thetaDot     = 0
    self.tPlot        = []
    self.thetaPlot    = []
    self.thetaDotPlot = []
    self.q_motPlot    = []
    self.p_APlot      = []
    self.p_BPlot      = []
    self.p_pPlot      = []
    self.adPlot       = []

    self.t = 0

  def moveWinch(self, ad, p_p):
    self.q_PA = cd*ad*(2/rho_o*abs(p_p-self.p_A))**0.5
    self.q_BT = cd*ad*(2/rho_o*abs(self.p_B))**0.5
    self.q_mot = self.thetaDot*self.D_m/(2*pi)

    self.pDot_A = beta_o*(self.q_PA - self.q_mot)/self.Va
    self.pDot_B = beta_o*(self.q_mot - self.q_BT)/self.Vb

    self.updateState(ad, p_p)

  def updateState(self, ad, p_p):
    self.thetaDotDot = (self.D_m*(self.p_A - self.p_B)/(2*pi) - self.Meff)/self.Jeff
    self.Meff = (self.m_pl*g*self.r_dr)/self.i_mot + self.thetaDot*self.fric
    self.thetaDot = self.thetaDot + self.thetaDotDot*dt
    self.theta = self.theta + self.thetaDot*dt
    self.t = self.t + dt
    self.p_A = self.p_A + self.pDot_A*dt
    self.p_B = self.p_B + self.pDot_B*dt

    self.tPlot.append(self.t)
    self.thetaPlot.append(self.theta)
    self.thetaDotPlot.append(self.thetaDot)
    self.p_BPlot.append(self.p_B/1e5)
    self.p_APlot.append(self.p_A/1e5) 
    self.p_pPlot.append(p_p/1e5)
    self.adPlot.append(ad)

  def plotData(self, *args):
    index = 0
    for arg in args:
      plt.plot(self.tPlot, arg, linewidth = 1, label = index)
      plt.legend()
      index += 1
    plt.draw()
    plt.show()
