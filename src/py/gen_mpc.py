import casadi.casadi as cs
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

def discrete_dynamics(xk,uk,dt, xdot):
    f = lambda x,u: cs.vertcat(*xdot(x,u))

    steps = 1
    d= dt/steps
    x = xk
    for _ in range(steps):
      k1 = f(x, uk)
      k2 = f(x + d/2 * k1, uk)
      k3 = f(x + d/2 * k2, uk)
      k4 = f(x + d * k3, uk)
      x = x + d/6 * (k1 + 2*k2 + 2*k3 + k4)

    return x

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
    N = 5
    K = N + 1
    dt = 0.04

    PROGRESS_WEIGHT = 5
    POS_ERR_WEIGHT = 50
    VEL_WEIGHT = 10
    THETA_WEIGHT = 100
    V_CONSTRAINT_TOLERANCE = 0.1

    def __init__(self):
        opti = cs.Opti()
        self.opti = opti
        self.x = [opti.variable(self.nx) for _ in range(self.K)]
        self.u = [opti.variable(self.nu) for _ in range(self.N)]
        ng = []

        self.p = opti.parameter(7)

        self.xdot = lambda x,u: mecanumDrivebase(MotorModelParameters(self.p[0],self.p[1]),self.p[2],self.p[3],self.p[4],self.p[5],self.p[6],x,u)

        self.p_t = opti.parameter(8)

        self.u_last = opti.parameter(4)

        self.t = Waypoint([self.p_t[0],self.p_t[1]],self.p_t[2],[self.p_t[3],self.p_t[4]],self.p_t[5],self.p_t[6],self.p_t[7])

        self.x0 = opti.parameter(self.nx)   

        self.u_initial = opti.parameter(self.N*self.nu)
        self.x_initial = opti.parameter((self.N+1)*self.nx)
    
        for k in range(self.K):
            g = 0
            if k < self.K - 1:
                # dynamics constraints
                opti.subject_to(self.x[k+1] == discrete_dynamics(self.x[k],self.u[k],self.dt, self.xdot))

                # input constraints
                for i in range(self.nu):
                    c = (-1.0 <= (self.u[k][i] <= 1.0))
                    opti.subject_to(c)
                    g += c.nnz()

            #if k > self.K/2:
                #c = (self.t_v[0]**2 + Optimizer.V_CONSTRAINT_TOLERANCE > self.x[k][0]**2)
                #opti.subject_to(c)
                #g += c.nnz()

                #c = ((self.t_v[1])**2 + Optimizer.V_CONSTRAINT_TOLERANCE > self.x[k][1]**2)
                #opti.subject_to(c)
                #g += c.nnz()

            if k == 0:
                for i in range(self.nx):
                    c = self.x[0][i] == self.x0[i]
                    # initial state constraints
                    opti.subject_to(c)
                    g += c.nnz()
            ng.append(g)

        j = 0.0
        for i in range(self.nu):
            j += (self.u[0][i]-self.u_last[i])**2
        for k in range(self.K):
            if k<self.K-2:
                for i in range(self.nu):
                    j += (self.u[k][i]-self.u[k+1][i])**2
            j += self.cost(self.x[k],self.t)

        opti.minimize(j)

        # for k in range(self.K):
        #     if k < self.K-1:
        #         # initial input
        #         opti.set_initial(self.u[k], cs.vef.K):
        #     opti.set_initiartcat(0,0,0,0))

        # Solver options - try fatrop first, fallback to ipopt
        opti.solver('fatrop', {
            "expand": True, 
            "fatrop.tol": 1e-6,
            "fatrop.print_level":0,
            "print_time": False,
            "jit": False
        })
        
        x = cs.vertcat(*self.x)
        u = cs.vertcat(*self.u)

        opti.set_initial(x, [1.23456 for i in range(self.nx*self.K)])
        
        self.f = self.opti.to_function('f',[
            self.p_t,
            self.x0, 
            self.u_last, 
            self.p,
            ],[x,u])
        # ipopt with lbfgs and a large memory size
        #opti.solver("ipopt", {"ipopt.hessian_approximation": "limited-memory", "ipopt.tol": 1e-6})
        print("Using FATROP solver") # Solver options - try fatrop first, fallback to ipopt

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
    o = Optimizer()

    _gen = cs.CodeGenerator("f.c", dict(with_header=True))
    _gen.add(o.f)

    print("Running casadi codegen...")
    _gen.generate(dir_path)
