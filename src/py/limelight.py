import numpy as np
import subprocess
import time
import os 

height, width = 960, 1280

addUser = False
startSolver = True

def runPipeline(image, llrobot):
    global addUser
    global startSolver
    if addUser:
        print("adding user")
        result = subprocess.run("echo \"22377\n22377\n\" | sudo adduser sigmacorns", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(str(result.stdout).replace('\\n', '\n').replace('\\t', '\t'))
        print(result)
        addUser = False
    if startSolver:
        print("stopping existing solver if it exists")
        
        result = subprocess.run("kill $(ps | awk '/solver/ && !/awk/ {print $1}') 2>/dev/null", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(str(result.stdout).replace('\\n', '\n').replace('\\t', '\t'))
        print(result)

        print("starting solver")

        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = "/home/sigmacorns/"

        subprocess.Popen(["/home/sigmacorns/solver", "5000"], env=env)

        startSolver = False
    return np.array([[]]), image, [0, 0, 0, 0, 0, 0, 0, 0]
