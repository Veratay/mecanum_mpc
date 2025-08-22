from casadi import MX
import casadi.casadi as cs
import rockit
import math

class MotorModelParameters:
  def __init__(self,free_speed,stall_torque):
    self.w_max = free_speed;
    self.t_max = stall_torque;

def torque(motorConstants, v, u):
  return motorConstants.t_max * (u - v/motorConstants.w_max)

def mecanumDrivebase(motorConstants, lx, ly, wheelRadius, weight, rotInertia, x, u):
  # x = [vx,vy,omega, x,y,theta]
  # u = [inputs(4)] FL, BL, BR, FR
  vx = x[0]*cs.cos(-x[5])-x[1]*cs.sin(-x[5])
  vy = x[0]*cs.sin(-x[5])+x[1]*cs.cos(-x[5])
  motorVels = [
    (vx-vy-(lx+ly)*x[2])/wheelRadius,
    (vx+vy-(lx+ly)*x[2])/wheelRadius,
    (vx-vy+(lx+ly)*x[2])/wheelRadius,
    (vx+vy+(lx+ly)*x[2])/wheelRadius,
  ]

  fs = [torque(motorConstants,motorVels[i],u[i])/wheelRadius for i in range(4)]
  dV = [
    (fs[0]+fs[1]+fs[2]+fs[3])/weight,
    (-fs[0]+fs[1]-fs[2]+fs[3])/weight,
    (-fs[0]-fs[1]+fs[2]+fs[3])*cs.sqrt(lx**2+ly**2)/rotInertia,
  ]

  dx = [
    dV[0]*cs.cos(x[5])-dV[1]*cs.sin(x[5]),
    dV[0]*cs.sin(x[5])+dV[1]*cs.cos(x[5]),
    dV[2],
    x[0],
    x[1],
    x[2]
  ]

  return dx

def curve_err(p0, theta, r, x):
  cx = p0[0]+r*cs.sin(theta)
  cy = p0[1]-r*cs.cos(theta)
  dx = x[0]-cx
  dy = x[1]-cy
  drx = cs.cos(-theta)*dx - cs.sin(-theta)*dy
  dry = cs.sin(-theta)*dx + cs.cos(-theta)*dy

  progress = r*(cs.copysign(1,r)*math.pi/2.0-cs.atan2(dry,drx))
  err = cs.fabs(r)-cs.hypot(dx,dy)
  return [progress,err]
  
def line_err(p0, theta, x):
  dx = x[0]-p0[0]
  dy = x[1]-p0[1]
  return [cs.cos(-theta)*dx - cs.sin(-theta)*dy, cs.sin(-theta)*dx + cs.cos(-theta)*dy]

class Waypoint:
  def __init__(self, x, theta, v, w, tangent, r):
    self.x = x
    self.v = v
    self.theta = theta
    self.w = w
    self.tangent = tangent
    self.r = r

class Optimizer:
    nx = 6
    nu = 4
    N = 7
    K = N + 1
    dt = 0.04

    PROGRESS_WEIGHT = 5
    POS_ERR_WEIGHT = 50
    VEL_WEIGHT = 10
    THETA_WEIGHT = 100
    V_CONSTRAINT_TOLERANCE = 0.1

    def __init__(self, codegen_path):
        T = self.dt*self.K
        ocp = rockit.Ocp(T = T)
        self.x: MX = ocp.state(self.nx)
        self.u: MX = ocp.control(self.nu)
        self.x0: MX = ocp.register_parameter(cs.MX.sym('x0',self.nx))   
        self.p: MX = ocp.register_parameter(cs.MX.sym('p',7))
        self.t: MX = ocp.register_parameter(cs.MX.sym('t',8))
        self.u_last: MX = ocp.register_parameter(cs.MX.sym('u_last',4))

        ocp.set_value(self.x0, [0.0]*self.nx)
        ocp.set_value(self.p, [0.0]*7)
        ocp.set_value(self.t, [0.0]*8)
        ocp.set_value(self.u_last, [0.0]*4)

        ocp.set_der(self.x, cs.vertcat(*mecanumDrivebase(MotorModelParameters(self.p[0],self.p[1]),self.p[2],self.p[3],self.p[4],self.p[5],self.p[6],self.x,self.u)))

        self.target = Waypoint([self.t[0],self.t[1]],self.t[2],[self.t[3],self.t[4]],self.t[5],self.t[6],self.t[7])

        ocp.subject_to(ocp.at_t0(self.x) == self.x0)
        ocp.subject_to((([-1.]*self.nu)<self.u)<([1.]*self.nu))

        ocp.add_objective(ocp.sum(self.cost(self.x,self.target)))

        # ocp.set_initial(self.u, [0.0]*self.nu)

        ocp.method(rockit.external_method('fatrop', N=self.N, intg='rk'))
        ocp._method.set_name("./codegen")

        _, u_sampled = ocp.sample(self.u, grid = 'control-')
        self.ocp = ocp

        self.f = ocp.to_function('solve_mecanum', [ocp.value(self.x0), ocp.value(self.p), ocp.value(self.t), ocp.value(self.u_last)], [u_sampled])

    def cost(self,x,t):
        p = [x[3],x[4]]
        t_theta = t.theta + self.dt*t.w
        progress, normal_err = curve_err(t.x, t.tangent, t.r, p)
        #progress, normal_err = line_err(t.x, t.tangent, p)
        return Optimizer.PROGRESS_WEIGHT*-progress + Optimizer.POS_ERR_WEIGHT*normal_err**2 + Optimizer.VEL_WEIGHT*((t.v[0]-x[0])**2 + (t.v[1]-x[1])**2) + Optimizer.THETA_WEIGHT*(x[5]-t_theta)**2

if __name__ == "__main__":
    import sys
    import os

    dir_path = sys.argv[1]

    # Construct the full path to f.c in that directory
    print("Generated code will be saved to " + str(dir_path))
    o = Optimizer(dir_path)

    #o.ocp.solve()
