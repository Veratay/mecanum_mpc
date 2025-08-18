import numpy as np
import subprocess
import time
from ctypes import cdll, c_char_p, POINTER, c_double, c_int, c_void_p

height, width = 960, 1280

class Solver:
    def __init__(self, lib):
        self.lib = lib
        self._handle = lib.solver_create()

    def solve(self, target, x0, u_last, p, x_out, u_out):
        return self.lib.solver_solve(
            self._handle,
            target.ctypes.data_as(POINTER(c_double)),
            x0.ctypes.data_as(POINTER(c_double)),
            u_last.ctypes.data_as(POINTER(c_double)),
            p.ctypes.data_as(POINTER(c_double)),
            x_out.ctypes.data_as(POINTER(c_double)),
            u_out.ctypes.data_as(POINTER(c_double))
        )

    def __del__(self):
        if hasattr(self, "_handle") and self._handle:
            self.lib.solver_destroy(self._handle)

def loadLibrary():
    return cdll.LoadLibrary('/home/dolphinpod/libsolver.so')

addUser = False
logIP = False
solver = None

def runPipeline(image, llrobot):
    global addUser
    global logIP
    global lib
    global solver
    if addUser:
       result = subprocess.run("echo \"pass\npass\" | sudo adduser sigma sudo", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
       print(str(result.stdout).replace('\\n', '\n').replace('\\t', '\t'))
       print(result)
       addUser = False
    if logIP:
       result = subprocess.run("ip addr", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
       print(str(result.stdout).replace('\\n', '\n').replace('\\t', '\t'))
       print(result)
       logIP = False

    if solver is None:
        lib = loadLibrary()
        SolverHandle = c_void_p
        lib.solver_create.restype = SolverHandle
        lib.solver_destroy.argtypes = [SolverHandle]
        lib.solver_solve.argtypes = [
            SolverHandle,
            POINTER(c_double),  # target
            POINTER(c_double),  # x0
            POINTER(c_double),  # u_last
            POINTER(c_double),  # p
            POINTER(c_double),  # x_out
            POINTER(c_double)   # u_out
        ]
        lib.solver_solve.restype = c_int
        solver = Solver(lib)
    
    n = 7  # example sizes
    nu = 4
    nx = 6
    target = np.array([0.0,0.0,0.0,1.0,0.0,0.0,0.0,100.0], dtype=np.float64)
    x0 = np.zeros(nx, dtype=np.float64)
    u_last = np.zeros(nu, dtype=np.float64)
    p = np.array([96.66,7.8,0.27,0.27,0.1016,68.0389,6.0], dtype=np.float64)
    x_out = np.zeros((n+1)*nx, dtype=np.float64)
    u_out = np.zeros(n*nu, dtype=np.float64)

    t0 = time.perf_counter()

    solver.solve(target,x0,u_last,p,x_out,u_out)

    t1 = time.perf_counter()

    print("solved in " + str(t1-t0) + "s")

    print("u = " + str(u_out))
    
    return np.array([[]]), image, [0, 0, 0, 0, 0, 0, 0, 0]
