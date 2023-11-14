import spatialmath as sm
import numpy as np

if __name__ == '__main__':
    R = sm.SO3([[0.9978, 0.02239, -0.06253],[0.007113, -0.9721, -0.2346],[-0.06604, 0.2336, -0.9701]])
    rpy = R.rpy()
    deg = [np.rad2deg(ang) for ang in rpy]
    print(deg)