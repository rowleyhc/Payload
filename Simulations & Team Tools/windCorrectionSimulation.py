'''
See documentation here: https://docs.google.com/document/d/1soUME8JDSpf028hsgl010TmuEHOHm2ZJCv7ecYDvrWE/edit
'''


import matplotlib.pyplot as plt
import math
import random as rand


mass=1 #kg
t=0 #s
tStep=0.005 #s

vSpeed=2
vsError=0.8
pos=[0,0]
target=[0,5]
windEst=[0,0]
windAct=[.3,0]
windVariationSpeed=0.1

position=[0,0]
windx=[]
windy=[]
times=[]
headings=[]
xs=[0]
ys=[0]
vxs=[]
vys=[]
posPrev=[0,0]
g=[0,0]

weightFactor=1

#Dumb state values (no wind correction)
d_position=[0,0]
d_xs=[0]
d_ys=[0]
d_vxs=[]
d_vys=[]
d_posPrev=[0,0]


while t<5:

	#Update time
	t+=tStep
	times.append(t)

	#Update wind
	windAct[0]+=windVariationSpeed*(2*rand.random()-1)
	windAct[1]+=windVariationSpeed*(2*rand.random()-1)
	windx.append(windAct[0])
	windy.append(windAct[1])

	#Calculate heading vector for smart system
	h=[target[0]-position[0],target[1]-position[1]]
	hMag=(h[0]**2+h[1]**2)**0.5
	h=[h[0]/hMag,h[1]/hMag]

	#Calculate heading angle for dumb system
	d_heading=math.atan2(target[1]-d_position[1],target[0]-d_position[0])

	#Determine wp using estimated wind vector 
	wpconst=((windEst[0]*h[0]+windEst[1]*h[1])/(h[0]**2+h[1]**2))
	wp=[h[0]*wpconst,h[1]*wpconst]

	#Determine from wp and wind estimate
	wc=[windEst[0]-wp[0],windEst[1]-wp[1]]

	#Create g based on wind strength
	if (windEst[0]**2+windEst[1]**2) < vSpeed**2:
		hPrimeMag=(vSpeed**2-(wc[0]**2+wc[1]**2))**0.5
		g[0]=h[0]*hPrimeMag-wc[0]
		g[1]=h[1]*hPrimeMag-wc[1]
	else: #This is primarily a saftey thing, not much control theory behind it
		g=[-windEst[0],-windEst[1]]


	#Calculate heading angle from g
	heading=math.atan2(g[1],g[0])

	#Update velocities
	velocity=[windAct[0]+vsError*vSpeed*math.cos(heading),windAct[1]+vsError*vSpeed*math.sin(heading)]
	d_velocity=[windAct[0]+vsError*vSpeed*math.cos(d_heading),windAct[1]+vsError*vSpeed*math.sin(d_heading)]

	#Update positions
	posPrev=position.copy()
	position[0]+=velocity[0]*tStep
	position[1]+=velocity[1]*tStep
	d_position[0]+=d_velocity[0]*tStep
	d_position[1]+=d_velocity[1]*tStep

	#Save values to arrays
	xs.append(position[0])
	ys.append(position[1])
	vxs.append(velocity[0])
	vys.append(velocity[1])
	d_xs.append(d_position[0])
	d_ys.append(d_position[1])
	d_vxs.append(d_velocity[0])
	d_vys.append(d_velocity[1])
	
	#Update the wind estimate using 
	newWindEst=[((position[0]-posPrev[0])/tStep)-vSpeed*math.cos(heading),((position[1]-posPrev[1])/tStep)-vSpeed*math.sin(heading)]
	windEst=[(1-weightFactor)*windEst[0]+weightFactor*newWindEst[0],(1-weightFactor)*windEst[1]+weightFactor*newWindEst[1]]
	



#Plot wind canceling and dumb trajectories
plt.plot(xs,ys, ls="-", color='red')
plt.plot(d_xs,d_ys, ls=":", color='blue')
#plt.plot(times,velocities, ls=":", color='blue')
plt.show()