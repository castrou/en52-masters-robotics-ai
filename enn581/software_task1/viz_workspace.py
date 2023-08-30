import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    x = []
    y = []
    z = []
    with open('workspace.csv', 'r') as csv:
        for i, line in enumerate(csv.readlines()):
            if (i % 2500 == 0):
                x_coord, y_coord, z_coord = [coord.strip() for coord in line.split(',')] 
                x_coord = np.array(x_coord, dtype=float)
                y_coord = np.array(y_coord, dtype=float)
                z_coord = np.array(z_coord, dtype=float)
                # print(f"{x_coord}, {y_coord}, {z_coord}")
                x.append(x_coord)
                y.append(y_coord)
                z.append(z_coord)
    
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter(x, y, z, marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()
    