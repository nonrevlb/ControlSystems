import sys
import numpy as np
import param as P

def getForces(y_r,y):
  # y_r is the referenced input
  # y is the current state
  z_r = y_r[0]
  z = y[0]
  zdot = y[3]

  h_r = y_r[1]
  h = y[1]
  hdot = y[4]

  th = y[2]
  thdot = y[5]

  F = P.h_kp*(h_r-h)-P.h_kd*hdot + P.F0
  th_r = P.z_kp*(z_r-z)-P.z_kd*zdot
  T = P.th_kp*(th_r-th)-P.th_kd*thdot

  Fr = F/2.0 + T/2.0/P.d
  Fl = F/2.0 - T/2.0/P.d
  Fr_sat = saturate(P.Fr_max,Fr)
  Fl_sat = saturate(P.Fl_max,Fl)
  return [Fr_sat,Fl_sat,th_r]

def saturate(limit,u):
    if abs(u) > limit:
      u = limit*np.sign(u)
    return u
