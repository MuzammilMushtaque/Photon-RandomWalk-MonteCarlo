import numpy as np
from numpy.random import random as rand
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

Nbins = 500
Nbinspl = 501
c = 2.99e10 #cm/s
ALIVE = 1
DEAD = 0
Threshold = 0.01
Chance = 0.1
cos90D = 1e-6
ONE_MINUS_COSZERO = 1e-12
mua = 1.673 #cm-1
mus = 312.0 #cm-1
g = 0.90
Nphotons = 1#10
radial_size = 0.1  # 2 cm
NR = Nbins
dr = radial_size/NR # cm
albedo = mus/(mus+mua)
EPS = 1.0e-6


class Photon():
    def __init__(self,u,pos, W, A):
        self.ux = u[0]
        self.uy = u[1]
        self.uz = u[2]
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.W = W
        self.A = A[0]   # list of zeros used for photon weight that drops in a specific x and y 
    def HOP(self):
        # take new step .....
        s = - np.log(rand(1))/(mua+mus)
        
        self.x += s*self.ux
        self.y += s*self.uy
        self.z += s*self.uz
        xPos = self.x
        yPos = self.y
        zPos = self.z
        return (xPos,yPos,zPos)    # not sure about 3 returns...?????
        
    def DROP(self):
        # drop photon weight into local bin .........
        absorb = self.W * (1.0-albedo)
        self.W -= absorb
        
        # Photon weight account in Cylindrical Coordinate here .....
        r = (self.x**2 + self.y**2)**0.5
        ir = int(r/dr)+1
        if ir >= NR:
            ir = NR-1
        else:
            ir = ir
        #print ir        
        self.A[ir] += absorb   #DROP absorbed weight into bin....
        Photon_weight = self.A[ir]
        return (ir, Photon_weight)
        
    def SPIN(self):
        # scatter photon into new trajectory defined by theta and psi ..........
        # we get cos(theta) and sin(theta) from below .... 
        # Sample theta .....
        if g == 0:
            cos_theta = 2.0*rand(1) -1.0
        else:
            temp = (1.0-g**2)/(1.0-g+2.0*g*rand(1))
            cos_theta = (1.0+g**2-temp**2)/2.0*g
        sin_theta = (1.0 - cos_theta**2)**0.5
        # Sample psi...
        psi = 2.0 * np.pi * rand(1)
        cos_psi = np.cos(psi)
        if psi < np.pi:
            sin_psi = (1.0-cos_psi**2)**0.5
        else:
            sin_psi = -(1.0-cos_psi**2)**0.5
        
        # New Trajectory .....
        temp = (1.0 - self.uz**2)**0.5
        if np.abs(temp) > EPS:
            uxx = sin_theta*(self.ux*self.uz*cos_psi - self.uy*sin_psi)/temp + self.ux*cos_theta
            uyy = sin_theta*(self.uy*self.uz*cos_psi + self.ux*sin_psi)/temp + self.uy*cos_theta
            uzz = -sin_theta*cos_psi*temp + self.uz*cos_theta
        else:
            uxx = sin_theta * cos_psi
            uyy = sin_theta * sin_psi
            if self.uz >= 0:
                uzz = cos_theta
            else:
                uzz = - cos_theta
        # Update Trajectory ...................
        self.ux = uxx
        self.uy = uyy
        self.uz = uzz



class RunPhotonPackage():
    def __init__(self,radial_size,Nphotons):
        self.radial_size = radial_size
        self.Nphotons = Nphotons
    def RunPhoton(self):
        Dist_Pos = np.zeros((3, self.Nphotons))
        
        # loop over number of photons.....
        for i in range(self.Nphotons):
            #initiate photon position to origin .....
            pos_init = [0,0,0] # x=y=z=0   initially...
            # theta and psi .... randomly set photon trajectory to yield an isotropic source .......
            k_init = [0,0,0]
            cos_theta = 2.0*rand(1) -1.0
            sin_theta = (1-cos_theta**2)**0.5
            psi = 2.0*np.pi*rand(1)
            k_init[0] = sin_theta * np.cos(psi)
            k_init[1] = sin_theta * np.sin(psi)
            k_init[2] = cos_theta
            W = 1.0
            Ccly = np.zeros((1, Nbins))
            newPhoton = Photon(k_init,pos_init, W, Ccly)
            newposition = [0.,0.,0.] # here we only apply that simulation continue propagating photon in z-axis would reach at max length....
            
            # Save the trajectories in seperate files
            '''
            while ((newposition[2] >= -2) and (newposition[2] <= self.radial_size)):
                newposition = newPhoton.HOP()
                newdrop = newPhoton.DROP()
                newscatter = newPhoton.SPIN()
                f = open('test_one_photon_A_'+str(i)+'.dat',"a")
             #   f.write(str(int(newdrop[0]))+'   '+str(np.float64(newdrop[1]))+'\n')
                f.write(str(np.float64(newposition[0]))+'    '+str(np.float64(newposition[1]))+'    '+str(np.float64(newposition[2]))+'\n')
                f.close()
            '''

if __name__ == "__main__":
    resulted_data = (RunPhotonPackage(radial_size, Nphotons).RunPhoton())   # class



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.plot3D(0,0,0,'ok')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
for k in range(0,10,1):
    data = np.loadtxt('one_photon_trajectory_scatter/one_photon_pos_'+str(k)+'.dat')
    x,y,z=[],[],[]
    for i in range(len(data)):
        x.append(data[i][0])
        y.append(data[i][1])
        z.append(data[i][2])


    ax.plot3D((x),(y),(z))
    plt.pause(0.5)
plt.show()


