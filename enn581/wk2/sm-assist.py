import spatialmath as sm
import roboticstoolbox as rtb

def q1():
    Tx = sm.SE3.Tx(3.0)
    Ty = sm.SE3.Ty(5.0)
    Tz = sm.SE3.Tz(2.0)
    Rz = sm.SE3.Rz(90, unit='deg')
    Ry = sm.SE3.Ry(-90, unit='deg')
    aTb = Tx @ Ty @ Tz @ Rz @ Ry
    print(aTb)
    
    
if __name__ == '__main__':
    print('--- Q1 ---')
    q1()