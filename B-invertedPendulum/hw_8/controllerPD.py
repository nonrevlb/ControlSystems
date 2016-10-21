import numpy as np 
import param as P

def saturate(limit,u):
  if abs(u) > limit:
    u = limit*np.sign(u)
  return u

def getForces(ref_inputs,states):

  z_r = ref_inputs[0]

  z = states[0]         
  th = states[1]
  zdot = states[2]
  thdot = states[3]

  th_r = P.z_kp*(z_r-z)-P.z_kd*zdot
  F_unsat = P.th_kp*(th_r-th)-P.th_kd*thdot
  F_sat = saturate(P.F_max,F_unsat)
    
  return [F_sat]





