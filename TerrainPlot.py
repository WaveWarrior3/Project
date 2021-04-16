import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d



def CreateTerrainMap(data):
    '''
    data is a (res x res x 3) tensor of distance data
    '''

    x = data[:,:,0]
    y = data[:,:,1]
    z = data[:,:,2]

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.scatter(x, y, z)

    plt.show()



def main():
    data = np.random.rand(10,10,3)
    CreateTerrainMap(data)



if __name__ == "__main__":
    main()