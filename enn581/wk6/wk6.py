import roboticstoolbox as rtb
import spatialmath as sm


def q5():
    q5bot = rtb.Robot(rtb.ETS([
        rtb.ET.Rz(),
        rtb.ET.tx(3),
        rtb.ET.Rz(),
        rtb.ET.tx(2),
        rtb.ET.Rz(),
        rtb.ET.tx(1)
    ]))
    print(q5bot)
    ee_init = sm.SE3.Tx(5) @ sm.SE3.Ty(1) @ sm.SE3.Rz(90, unit='deg')
    ee_goal = sm.SE3.Tx(-3) @ sm.SE3.Ty(3) @ sm.SE3.Rz(180, unit='deg')
    
    q0 = q5bot.ik_NR(ee_init)[0]
    q5bot.q = q0
    qf = q5bot.ik_NR(ee_goal)[0]
        
    
if __name__ == '__main__':
    q5()