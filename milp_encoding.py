from ortools.linear_solver import pywraplp
import numpy as np
import matplotlib.pyplot as plt


grid_size_x = 2
grid_size_y = 3
start = (0,0)
end  = (1,2)
obstacles = [(1,0)]


def get_outgoing(x,y,grid_size_x, grid_size_y):
	out_r = (None,None)
	out_y = (None,None)
	if (y+1)>=0 and (y+1)<grid_size_y:
		out_r = (x,y+1)
	if (x+1)>=0 and (x+1)<grid_size_x:
		out_y = (x+1,y)
	return out_r,out_y


def plot_discrete_grid(x,y,start,end,obstacles,sol_index):
	plt.plot(start[0], start[1], "^r")
	plt.plot(end[1],end[0], "^c")
	for i in range(y):
		for j in range(x):
			if (i == start[1] and j == start[0]):
				continue
			elif (i == end[1] and j == end[0]):
				continue
			else:
				plt.plot([i],[j],".b")
	if obstacles:
		for obs in obstacles:
			plt.plot(obs[1], obs[0], ".k")
	for i in sol_index :
		x1,y1,x2,y2 = get_index(i)
		plt.arrow(x1,y1,x2 - x1,y2 -y1)
	plt.gca().invert_yaxis()
	plt.xticks(np.arange(0, y, 1))
	plt.yticks(np.arange(0, x, 1))
	plt.grid()	
	plt.savefig('grid.png')


def get_index(a):
	y1 = int((a[0]/grid_size_y))
	x1 = a[0] - (y1*grid_size_y)
	y2 = int((a[1]/grid_size_y))
	x2 = a[1] - (y2*grid_size_y)
	return x1,y1,x2,y2


flat_grid_size = grid_size_x*grid_size_y
adjacency = np.zeros((flat_grid_size,flat_grid_size))
for i in range(grid_size_x):
	for j in range(grid_size_y):
	
		edges = get_outgoing(i,j,grid_size_x, grid_size_y)
	
		for e in edges:
			if e[0] != None and e[1] != None:
				row_major = e[0]*grid_size_y + e[1]
		
				adjacency[i*grid_size_y+j][row_major] = 1

solver = pywraplp.Solver('SolveIntegerProblem',pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
variable_list= [[0] * flat_grid_size for i in range(flat_grid_size)]
for i in range(flat_grid_size):
	for j in range(flat_grid_size):
			variable_list[i][j] = solver.IntVar(0,1, 'b_'+str(i)+'_'+str(j))
variable_list = np.array(variable_list)


for i in obstacles:
	node = i[0]*grid_size_y + i[1]
	incoming = adjacency[:,node]
	for j in range(len(incoming)):
		incoming[j] = 0

constraints = [0]*(flat_grid_size) #>=0
constraints_2 = [0]*(flat_grid_size) #<=0
variables_used = []

for i in range(flat_grid_size):

	if i == start[0]*grid_size_y + start[1]:
		constraints[i] = solver.Constraint(1,solver.infinity())
		constraints_2[i] = solver.Constraint(-solver.infinity(),1)
		for j in range(flat_grid_size):
			if adjacency[i][j]==1:
				print("start",variable_list[i][j])
				constraints[i].SetCoefficient(variable_list[i][j],1)
				constraints_2[i].SetCoefficient(variable_list[i][j],1)
				variables_used.append(variable_list[i][j])
	elif i == end[0]*grid_size_y + end[1]:
		constraints[i] = solver.Constraint(1,solver.infinity())
		constraints_2[i] = solver.Constraint(-solver.infinity(),1)
		for j in range(flat_grid_size):

			if adjacency[:,end[0]*grid_size_y + end[1]][j] == 1:
				print("end",variable_list[:,end[0]*grid_size_y + end[1]][j])
				constraints[i].SetCoefficient(variable_list[:,end[0]*grid_size_y + end[1]][j],1)
				constraints_2[i].SetCoefficient(variable_list[:,end[0]*grid_size_y + end[1]][j],1)
				variables_used.append(variable_list[:,end[0]*grid_size_y + end[1]][j])
	else:
		constraints[i] = solver.Constraint(0,solver.infinity())
		constraints_2[i] = solver.Constraint(-solver.infinity(),0)
		for j in range(flat_grid_size):
			if adjacency[:,i][j] == 1:
				print("incoming nodes to ",i,variable_list[:,i][j])
				constraints[i].SetCoefficient(variable_list[:,i][j],1)
				constraints_2[i].SetCoefficient(variable_list[:,i][j],1)
				variables_used.append(variable_list[:,i][j])
		for j in range(flat_grid_size):
			if adjacency[i:i+1,][0][j] == 1:
				print("outgoing nodes from ",i,variable_list[i:i+1,][0][j])
				constraints[i].SetCoefficient(variable_list[i:i+1,][0][j],-1)
				constraints_2[i].SetCoefficient(variable_list[i:i+1,][0][j],-1)
				variables_used.append(variable_list[i:i+1,][0][j])


objective = solver.Objective()
variables_used = list(set(variables_used))
for v in variables_used:
	objective.SetCoefficient(v,1) 
objective.SetMinimization()

status = solver.Solve()
solution_variables = []
if status == pywraplp.Solver.OPTIMAL:
	print('Solution:')
	print('Objective value =', solver.Objective().Value())
	print("Printing Solution for the variables used")
	for v in variables_used:
		print('%s = %d'%(v, v.solution_value()))
		if v.solution_value() == 1:
			solution_variables.append(v)
	solution_index = []
	for s in solution_variables:
		tok = str(s).split('_')
		solution_index.append([int(tok[-2]),int(tok[-1])])
	print(solution_index)
	plot_discrete_grid(grid_size_x,grid_size_y,start,end,obstacles,solution_index)
