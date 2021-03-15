from ortools.linear_solver import pywraplp
import numpy as np


grid_size_x = 4
grid_size_y = 4

def get_outgoing(x,y,grid_size_x, grid_size_y):
	out_r = (None,None)
	out_y = (None,None)
	if (y+1)>=0 and (y+1)<grid_size_y:
		out_r = (x,y+1)
	if (x+1)>=0 and (x+1)<grid_size_x:
		out_y = (x+1,y)
	return out_r,out_y


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

start = (0,0)
end  = (1,2)

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
  #print(adjacency[i])



objective = solver.Objective()
variables_used = list(set(variables_used))
for v in variables_used:
	objective.SetCoefficient(v,1) 
objective.SetMinimization()

status = solver.Solve()
if status == pywraplp.Solver.OPTIMAL:
		print('Solution:')
		print('Objective value =', solver.Objective().Value())
		print("Printing Solution for the variables used")
		for v in variables_used:
			print('%s = %d'%(v, v.solution_value()))
			
