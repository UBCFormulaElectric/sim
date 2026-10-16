import numpy as np
from forcespro_helper import forcespro
import get_userid

N = 10
nx = 3 # state variable count
nu = 2 # control variable count

# costs on control
R = np.zeros((nu,nu))
# costs on state
P = np.zeros((nx,nx)) # cost on state at final stage
Q = np.zeros((nx,nx))

A = np.zeros((nx,nx)) # transition on state
B = np.zeros((nx,nu)) # transition on control

umin = np.zeros(nu)
umax = np.zeros(nu)
xmin = np.zeros(nx)
xmax = np.zeros(nx)

# FORCESPRO multistage form
# assume variable ordering zi = [u{i-1}, x{i}] for i=1...N
stages = forcespro.MultistageProblem(N)

# get stages struct of length N
for i in range(N):
	# dimensions
	stages.dims[i]['n'] = nx+nu # number of stage variables (dimension of z)
	stages.dims[i]['r'] = nx    # number of equality constraints
	stages.dims[i]['l'] = nx+nu # number of lower bounds
	stages.dims[i]['u'] = nx+nu # number of upper bounds
	stages.dims[i]['p'] = 0     # number of polytopic constraints
	stages.dims[i]['q'] = 0     # number of quadratic constraints

	# cost
	if i == N-1:
		# R P blockmatrix
		stages.cost[i]['H'] = np.vstack((
			np.hstack((R,np.zeros((nu,nx)))),
			np.hstack((np.zeros((nx,nu)),P))
		))
	else:
		# R Q blockmatrix
		stages.cost[i]['H'] = np.vstack((
			np.hstack((R,np.zeros((nu,nx)))),
			np.hstack((np.zeros((nx,nu)),Q))
		))
	stages.cost[i]['f'] = np.zeros((nx+nu, 1)) # linear cost terms

	# # lower bounds
	stages.ineq[i]['b']['lbidx'] = range(1,nu+nx+1) # lower bound acts on these indices
	stages.ineq[i]['b']['lb'] = np.concatenate((umin,xmin),0) # lower bound for this stage variable
	# # upper bounds
	stages.ineq[i]['b']['ubidx'] = range(1,nu+nx+1) # upper bound acts on these indices
	stages.ineq[i]['b']['ub'] = np.concatenate((umax,xmax),0) # upper bound for this stage variable

	# # equality constraints
	if i < N-1:
		stages.eq[i]['C'] = np.hstack((np.zeros((nx,nu)),A))
	if i > 0:
		stages.eq[i]['c'] = np.zeros(nx)
	stages.eq[i]['D'] = np.hstack((B,-np.eye(nx))) # note that it is I rather than zeros, because equality constraint for state transition

# RHS of first eq. constr. is a parameter: stages(1).eq.c = -A*x0
# read 8.10
stages.newParam('minusA_times_x0', [1], 'eq.c')
# define output of the solver
# read 8.11
stages.newOutput('u0', 1, range(1, nu + 1))

codeoptions = forcespro.CodeOptions('solver_name')
# codeoptions.platform = '' # to specify the target platform
codeoptions.printlevel = 0 # optional, on some platforms printing is not supported
codeoptions.cleanup = 0 # to keep auxiliary source files (not needed for low-level interface)
stages.codeoptions = codeoptions
stages.generateCode(get_userid.userid)