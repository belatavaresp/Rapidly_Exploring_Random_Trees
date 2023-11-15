# %%
import plotly.graph_objects as go
import numpy as np
import math
import random

# %% [markdown]
# ## Environment 

# %%

class Environment:
    def __init__(self, obstacles, xmin, xmax, ymin, ymax, zmin, zmax):
        self.obstacles = np.array(obstacles)
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.zmin = zmin
        self.zmax = zmax
    
    def obsCollision(self, point1, point2):
        collision = 1 #assume no collision
        for obs in range(len(self.obstacles)):
            oXmin, oXmax, oYmin, oYmax = self.obstacles[obs][0], self.obstacles[obs][1], self.obstacles[obs][2], self.obstacles[obs][3]
            for u in np.linspace(0,1,101):
                x = point1[0] * u + point2[0] * (1 - u)
                y = point1[1] * u + point2[1] * (1 - u)
                if(x >= oXmin) and (x <= oXmax) and (y >= oYmin) and (y <= oYmax):
                    #path is inside an obstacle boundary
                    collision = 0
                    break
            if collision == 0: break
        return collision
        
    def freeSample(self):
        n = tree.size() - 1
        (x, y, z) = (tree.x[n], tree.y[n], tree.z[n]) #last node added to the tree
        for obs in range(len(self.obstacles)):
            oXmin, oXmax, oYmin, oYmax = self.obstacles[obs][0], self.obstacles[obs][1], self.obstacles[obs][2], self.obstacles[obs][3]
            if(x >= oXmin) and (x <= oXmax) and (y >= oYmin) and (y <= oYmax) and (z >= self.zmin) and (z <= self.zmax):
                tree.remove(n) #not a valid sample, should be removed of the tree
                return 0

    def goalRange(self):
        n = tree.size() - 1
        (x, y, z) = (tree.x[n], tree.y[n], tree.z[n]) #last node added to the tree
        if(x >= goalXmin) and (x <= goalXmax) and (y >= goalYmin) and (y <= goalYmax) and (z >= goalZmin) and (z <= goalZmax):
            return 1
        else:
            return 0
        
    def isFreeXY(self, x, y):
        for obs in range(len(self.obstacles)):
            oXmin, oXmax, oYmin, oYmax = self.obstacles[obs][0], self.obstacles[obs][1], self.obstacles[obs][2], self.obstacles[obs][3]
            if(x >= oXmin) and (x <= oXmax) and (y >= oYmin) and (y <= oYmax):
                return 0


# %% [markdown]
# ## RRT

# %%
class RRT3d:
    def __init__(self, start):
        (x,y,z) = start
        self.x = np.array([x])
        self.y = np.array([y])
        self.z = np.array([z])
        self.parent = np.array([0])
        
    def distance(self, n1, n2):
        p1 = np.array([self.x[n1], self.y[n1], self.z[n1]])
        p2 = np.array([self.x[n2], self.y[n2], self.z[n2]])

        dist = np.linalg.norm(p1 - p2)

        return dist
    
    def size(self):
        return len(self.x)
    
    def add(self, n, x, y, z):
        self.x = np.insert(self.x, n, x)
        self.y = np.insert(self.y, n, y)
        self.z = np.insert(self.z, n, z)

    # Remove node
    def remove(self, n):
        self.x = np.delete(self.x, n)
        self.y = np.delete(self.y, n)
        self.z = np.delete(self.z, n)
        
    def addEdge(self, parent, child):
        self.parent = np.insert(self.parent, child, parent)

    def removeEdge(self, n):
        self.parent = np.delete(self.parent, n)
        
    def step(self, near, rand):
        d = self.distance(near, rand)
        if d > stepSize:
            u = stepSize / d
            (xnear, ynear, znear) = (self.x[near], self.y[near], self.z[near])
            (xrand, yrand, zrand) = (self.x[rand], self.y[rand], self.z[rand])
            (px, py, pz) = (xrand - xnear, yrand - ynear, zrand - znear)
            theta = math.atan2(py, px)
            x = xnear + stepSize * math.cos(theta)
            y = ynear + stepSize * math.sin(theta)
            alpha = math.atan2(pz, y)
            z = znear + stepSize * math.sin(alpha)
            
            # Remove the old node and add the updated node (between rand and near)
            self.remove(rand)
            self.add(rand, x, y, z)
            
    def near(self, n):
        min = self.distance(0,n)
        nearNode = 0
        for i in range(0,n):
            if self.distance(i, n) < min:
                min = self.distance(i, n)
                nearNode = i
        return nearNode
    
    def connect(self, node1, node2):
        point1 = (self.x[node1],self.y[node1],self.z[node1])
        point2 = (self.x[node2],self.y[node2],self.z[node2])
        n = tree.size() - 1 
        if environment.obsCollision(point1, point2) == 0:
            self.remove(node2) #can't connect (obstacle collision detected)
        else:
            self.addEdge(node1, node2)
            
    def expand(self):
        x = random.uniform (environment.xmin, environment.xmax)
        y = random.uniform (environment.ymin, environment.ymax)
        z = random.uniform (environment.zmin, environment.zmax)
        n = self.size()
        self.add(n, x, y, z)
        if environment.freeSample()!=0: #lattest added node must be valid
            nearest = self.near(n)
            self.step(nearest, n)
            self.connect(nearest, n)
            
    def bias(self):
        n = self.size()
        self.add(n, goalX, goalY, goalZ)
        nearest = self.near(n)
        self.step(nearest, n)
        self.connect(nearest, n)
    
    def clear(self, start):
        (x,y,z) = start
        self.x = np.array([x])
        self.y = np.array([y])
        self.z = np.array([z])
        self.parent = np.array([0])
    
    def pathToGoal(self):
        # find goal state
        goalStateIndices = np.where((self.x >= goalXmin) & (self.x <= goalXmax) &
                                    (self.y >= goalYmin) & (self.y <= goalYmax) &
                                    (self.z >= goalZmin) & (self.z <= goalZmax))[0]

        if len(goalStateIndices) == 0:
            return  # No goal state found

        self.goalstate = goalStateIndices[0]

        # add goal state and its parent nodes to the path
        self.path = [self.goalstate]
        newPos = self.parent[self.goalstate]

        # keep adding parents
        while newPos != 0:
            self.path.append(newPos)
            newPos = self.parent[newPos]

        # add start state
        self.path.append(0)

    def prun(self):
        s = 0
        e = 2
        self.tpath = []
        self.tpath.append(self.path[s])
        for e in range(len(self.path)-1):
            p1=self.x[self.path[s]],self.y[self.path[s]],self.z[self.path[s]]
            p2=self.x[self.path[e]],self.y[self.path[e]],self.z[self.path[e]]
            if environment.obsCollision(p1,p2)==0: #CC is detected
                c=0
                self.tpath.append(self.path[e-1])
                s=e-1
        self.tpath.append(self.path[-1])	

    def waypoints(self):
        self.wayx = []
        self.wayy = []
        self.wayz = []
        self.newStart = []
        
        for i in range (0,len(self.tpath)-1):
            (x1,y1,z1)=(self.x[self.tpath[i]],self.y[self.tpath[i]],self.z[self.tpath[i]])
            (x2,y2,z2)=(self.x[self.tpath[i+1]],self.y[self.tpath[i+1]],self.z[self.tpath[i+1]])
            for j in range (0,101):
                dt=j/100.0
                x=x1*(dt)+x2*(1-dt)
                y=y1*(dt)+y2*(1-dt)
                z=z1*(dt)+z2*(1-dt)
                self.wayx.append(x)
                self.wayy.append(y)
                self.wayz.append(z)
				#measurement update
				#collision after update
                if environment.isFreeXY(x,y)==0:
					#point before collision is used for generating new plan
                    self.newStart.append(i*101+j-10)
                    break

    def sense(self):
        while len(self.newStart) != 0:
            # First observation state
            cn = self.newStart[0]
            cx, cy, cz = self.wayx[cn], self.wayy[cn], self.wayz[cn]
            self.clear((cx, cy, cz))

            # Balance between extending and biasing
            for i in range(0, maxIter):
                if i % 10 != 0:
                    self.expand()
                else:
                    self.bias()

                # Check if sample is in goal, if so STOP!
                if environment.goalRange() == 1:
                    break

            # Visualize the current state with a Plotly marker
            cn = self.newStart[0]
            cx, cy, cz = self.wayx[cn], self.wayy[cn], self.wayz[cn]

            fig.add_trace(go.Scatter3d(x=[cx], y=[cy], z=[cz], mode='markers', marker=dict(color='yellow', size=75, opacity=0.3)))

            # Find path in RRT
            self.pathToGoal()
            self.prun()

            # Display the updated plan under limited sensing
            draw()

            # Execute the waypoints
            self.waypoints()
            
    #draw tree
    def showTree(self):
        for i in range(0, self.number_of_nodes()):
            par = self.parent[i]
            x = [self.x[i], self.x[par]]
            y = [self.y[i], self.y[par]]
            z = [self.z[i], self.z[par]]
            fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', line=dict(color='blue', width=1)))
			
	#draw path 
    def showPath(self):
        for i in range(len(self.path) - 1):
            n1 = self.path[i]
            n2 = self.path[i + 1]
            x = [self.x[n1], self.x[n2]]
            y = [self.y[n1], self.y[n2]]
            z = [self.z[n1], self.z[n2]]
            fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines+markers', line=dict(color='red', width=2), marker=dict(size=4)))
			 
	#draw path to be executed
    def showTpath(self):
        for i in range(len(self.tpath) - 1):
            n1 = self.tpath[i]
            n2 = self.tpath[i + 1]
            x = [self.x[n1], self.x[n2]]
            y = [self.y[n1], self.y[n2]]
            z = [self.z[n1], self.z[n2]]
            fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines+markers', line=dict(color='green', width=2), marker=dict(size=5)))

# %%
startPoint = (100,100,100)

goalX = 5
goalY = 5
goalZ = 5

delta = 5 #maximum acceptable distance from goal location

goalXmin = goalX - delta
goalXmax = goalX + delta

goalYmin = goalY - delta
goalYmax = goalY + delta

goalZmin = goalZ - delta
goalZmax = goalZ + delta

stepSize = 5
maxIter = 5000

vx = [40, 40, 60, 60, 70, 70, 80, 80, 40, 40, 60, 60, 15, 15, 25, 25, 25, 25, 35, 35]
vy = [52, 100, 100, 52, 40, 60, 60, 40, 0, 48, 48, 0, 15, 30, 30, 15, 40, 60, 60, 40]
vz = [0, 100]
#hidden obstacle
hvx = []
#[15, 15, 25, 25, 25, 25, 35, 35]
hvy = []
#[15, 30, 30, 15, 40, 60, 60, 40]

obstacles = [
    (vx[0], vx[2], vy[0], vy[2], vz[0], vz[1]),  # Obstacle 1
    (vx[4], vx[6], vy[4], vy[6], vz[0], vz[1]),  # Obstacle 2
    (vx[8], vx[10], vy[8], vy[10], vz[0], vz[1]),  # Obstacle 3
    (vx[12], vx[14], vy[12], vy[14], vz[0], vz[1]),  # Obstacle 4
    (vx[16], vx[18], vy[16], vy[18], vz[0], vz[1]),  # Obstacle 5
]

tree = RRT3d(startPoint)
environment = Environment(obstacles, 0, 100, 0, 100, 0, 100)

fig = go.Figure()

# %%
# obst = obstacles
# print(obst)
# for i in range(len(obst)):
#     o1, o2, o3, o4 = obst[i][0], obst[i][1], obst[i][2], obst[i][3]
#     print(o1,o2,o3,o4)

# print()
# obs_num = len(vx)//4 #four vertices for each rectangular obstacle
# for i in range(1,obs_num+1):
#     xo=vx[4*(i-1)]
#     xm=vx[4*(i-1)+2]
#     yo=vy[4*(i-1)]
#     ym=vy[4*(i-1)+1]  
#     print(xo,xm,yo,ym)

# %%
def cubedraw(x, y, zl, zh, color, dash='solid'):
    for i in range(0, len(x) - 1):
        obx = [x[i], x[i + 1], x[i + 1], x[i], x[i]]
        oby = [y[i], y[i], y[i + 1], y[i + 1], y[i]]
        obz = [zl[i], zl[i], zh[i], zh[i], zl[i]]
        fig.add_trace(go.Scatter3d(x=obx, y=oby, z=obz, mode='lines', line=dict(color=color, dash=dash)))

# Draw trees and environment
def draw():
    # Draw obstacles
    for obs in range(len(environment.obstacles)):
        obx = [environment.obstacles[obs][0], environment.obstacles[obs][1], environment.obstacles[obs][1], environment.obstacles[obs][0], environment.obstacles[obs][0]]
        oby = [environment.obstacles[obs][2], environment.obstacles[obs][2], environment.obstacles[obs][3], environment.obstacles[obs][3], environment.obstacles[obs][2]]
        obzl = [environment.zmin] * 5  # Set the lower z-coordinate for the bottom of the obstacle
        obzh = [environment.zmax] * 5  # Set the upper z-coordinate for the top of the obstacle
        fig.add_trace(go.Scatter3d(x=obx, y=oby, z=obzl, mode='lines', line=dict(color='black')))
        fig.add_trace(go.Scatter3d(x=obx, y=oby, z=obzh, mode='lines', line=dict(color='black')))
        cubedraw(obx, oby, obzl, obzh, 'black')


    # Draw tree edges
    for i in range(0, len(tree.parent)):
        par = tree.parent[i]
        x = [tree.x[i], tree.x[par]]
        y = [tree.y[i], tree.y[par]]
        z = [tree.z[i], tree.z[par]]
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', line=dict(color='blue', width=1)))

    # Draw path
    for i in range(len(tree.path) - 1):
        n1 = tree.path[i]
        n2 = tree.path[i + 1]
        x = [tree.x[n1], tree.x[n2]]
        y = [tree.y[n1], tree.y[n2]]
        z = [tree.z[n1], tree.z[n2]]
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines+markers', line=dict(color='red', width=2), marker=dict(size=4)))

    # Draw path to be executed
    for i in range(len(tree.tpath) - 1):
        n1 = tree.tpath[i]
        n2 = tree.tpath[i + 1]
        x = [tree.x[n1], tree.x[n2]]
        y = [tree.y[n1], tree.y[n2]]
        z = [tree.z[n1], tree.z[n2]]
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines+markers', line=dict(color='green', width=2), marker=dict(size=5)))

    # Draw goal region
    fig.add_trace(go.Scatter3d(x=[goalXmin, goalXmin, goalXmax, goalXmax, goalXmin], y=[goalYmin, goalYmax, goalYmax, goalYmin, goalYmin], z=[goalZmin, goalZmin, goalZmin, goalZmin, goalZmin], mode='lines', line=dict(color='gray')))

    # Set the layout and display the plot
    fig.update_layout(scene=dict(aspectmode='cube'))
    fig.show()

# %%
def main():
    #balance between extending and biasing	
	for i in range(0,maxIter):
		if i%10!=0: tree.expand()
		else: tree.bias()
	#check if sample is in goal, if so STOP!		
		if environment.goalRange()==1:
			print ("found")
			break
	tree.pathToGoal()
	tree.prun()
		
	#display initial plan under limited sensing
	draw()
	
fig.write_html("RRT3d_Plot.html")

 
# run main when RRT is called
if __name__ == '__main__':
    main()


