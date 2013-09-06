from sot_robot.prologue import solver

def push(task):
    if isinstance(task,str): taskName=task
    elif "task" in task.__dict__:  taskName=task.task.name
    else: taskName=task.name
    solver.push(taskName)

def pop(task):
    if isinstance(task,str): taskName=task
    elif "task" in task.__dict__:  taskName=task.task.name
    else: taskName=task.name
    if taskName in solver.toList(): solver.rm(taskName)
    
def err2file(task,filename,mode="a"):
    out_file = open(filename,mode)
    out_file.write(task.name+"\n")
    err = [ [ 0 for i in range(4) ] for j in range(4) ]
    for i in range(4):
        for j in range(4):
            err[i][j] = task.featureDes.position.value[i][j] - task.feature.position.value[i][j]
        out_file.write(str(err[i])+"\n")
    out_file.close()
