import numpy as np
from test_controller_2.forces.forcespro_helper import forcespro
import test_controller_2.forces.get_userid as get_userid

N = 10
nx = 0
nu = 0
R = np.zeros((nu,nu))
P = np.zeros((nx,nx))
Q = np.zeros((nx,nx))
A = np.zeros((nx,nx))
B = np.zeros((nx,nu))
umin = np.zeros((nu,1))
umax = np.zeros((nu,1))
xmin = np.zeros((nx,1))
xmax = np.zeros((nx,1))

# FORCESPRO multistage form # assume variable ordering zi = [u{i-1}, x{i}] for i=1...N
stages = forcespro.MultistageProblem(N)

# get stages struct of length N
for i in range(N):
	# dimensions
	stages.dims[ i ]['n'] = nx+nu # number of stage variables
	stages.dims[ i ]['r'] = nx    # number of equality constraints
	stages.dims[ i ]['l'] = nx+nu # number of lower bounds
	stages.dims[ i ]['u'] = nx+nu # number of upper bounds
	
	# cost
	if ( i == N-1 ):
		stages.cost[ i ]['H'] = np.vstack((np.hstack((R,np.zeros((nu,nx)))), np.hstack((np.zeros((nx,nu)),P))))
	else:
		stages.cost[ i ]['H'] = np.vstack((np.hstack((R,np.zeros((nu,nx)))), np.hstack((np.zeros((nx,nu)),Q))))

	stages.cost[ i ]['f'] = np.zeros((nx+nu,1)) # linear cost terms
	# lower bounds
	stages.ineq[ i ]['b']['lbidx'] = range(1,nu+nx+1) # lower bound acts on these indices
	stages.ineq[ i ]['b']['lb'] = np.concatenate((umin,xmin),0) # lower bound for this stage variable
	# upper bounds
	stages.ineq[ i ]['b']['ubidx'] = range(1,nu+nx+1) # upper bound acts on these indices
	stages.ineq[ i ]['b']['ub'] = np.concatenate((umax,xmax),0) # upper bound for this stage variable

	# equality constraints
	if ( i < N-1 ):
		stages.eq[i]['C'] = np.hstack((np.zeros((nx,nu)),A))
	if ( i>0 ):
		stages.eq[i]['c'] = np.zeros((nx,1))
	stages.eq[i]['D'] = np.hstack((B,-np.eye(nx)))

# RHS of first eq. constr. is a parameter: stages(1).eq.c = -A*x0
stages.newParam('minusA_times_x0', [1], 'eq.c')
# define output of the solver
stages.newOutput('u0', 1, range(1,nu+1))

codeoptions = forcespro.CodeOptions('solver_name')
# codeoptions.platform = '' # to specify the target platform
codeoptions.printlevel = 0 # optional, on some platforms printing is not supported
codeoptions.cleanup = 0 # to keep auxiliary source files (not needed for low-level interface)
stages.codeoptions = codeoptions
stages.generateCode(get_userid.userid)