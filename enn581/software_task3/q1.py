import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt

class Cubic:
    def __init__(self, a0, a1, a2, a3):
        self.a0 = a0
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        
    def __str__(self):
        return f'[{self.a0, self.a1, self.a2, self.a3}]'
        
    def __call__(self, x):
        return self.a0 + self.a1*x + self.a2*x**2 + self.a3*x**3
    
    def dot(self,x):
        return self.a1 + 2*self.a2*x + 3*self.a3*x**2
    
    def ddot(self, x):
        return 2*self.a2 + 6*self.a3*x
    
def get_cubics(angle_list, duration):
    '''
    Get a list of cubics that connect the provided angle list (0 velocity at each via)\\
    
    Parameters\\
        angle_list <list<double>> : list of angles in degrees\\
        duration <list<double>> : list of durations (in order) of each cubic\\
        
    Returns\\
        cubics <list<Cubic>>\\
            
    REF: https://ocw.snu.ac.kr/sites/default/files/NOTE/Chap07_Trajectory%20generation.pdf
    '''
    cubic_starts = angle_list[:-1]
    cubic_ends = angle_list[1:]
    cubics = []

    for (start_ang, end_ang, dt) in zip(cubic_starts, cubic_ends, duration):
        theta_start = np.radians(start_ang)
        theta_end  = np.radians(end_ang)
        
        a0 = theta_start
        a1 = 0
        a2 = 3 * (theta_end - theta_start) / (dt**2)
        a3 = -2 * (theta_end - theta_start) / (dt**3)
        
        cubics.append(Cubic(a0, a1, a2, a3))
    return cubics


if __name__ == '__main__':
    # Initialise
    angles = [5, 25, 10, 30]
    traj_count = len(angles[:-1])
    durations = [2 for _ in range(traj_count)]
    
    # Get cubics 
    cubics = get_cubics(angles, durations)
    
    # Generate plot data
    dt = 0.01
    angle_series = []
    dangle_series = []
    ddangle_series = []
    cubic_timings = [0]
    for cubic, duration in zip(cubics, durations):
        cubic_timings.append(cubic_timings[-1] + duration)
        cubic_angs = []
        cubic_dangs = []
        cubic_ddangs = []
        T = np.arange(0, duration, dt)
        for t in T:
            cubic_angs.append(np.degrees(cubic(t)))
            cubic_dangs.append(np.degrees(cubic.dot(t)))
            cubic_ddangs.append(np.degrees(cubic.ddot(t)))
        angle_series.extend(cubic_angs)
        dangle_series.extend(cubic_dangs)
        ddangle_series.extend(cubic_ddangs)
    ## Final point
    angle_series.append(np.degrees(cubics[-1](durations[-1])))
    dangle_series.append(np.degrees(cubics[-1].dot(durations[-1])))
    ddangle_series.append(np.degrees(cubics[-1].ddot(durations[-1])))
    
    # Plot
    T = np.arange(0, sum(durations) + dt,dt)
    ## positions
    plt.figure(figsize=(12,8))
    plt.subplot(311)
    plt.plot(T, angle_series)
    plt.title('Joint Angle (deg) vs Time (s)')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (deg)')
    for i, timing in enumerate(cubic_timings):
        plt.plot(timing, angles[i], marker="o", markeredgecolor="blue", markerfacecolor='none')
    ## velocities
    plt.subplot(312)
    plt.plot(T, dangle_series)
    plt.title('Joint Velocity (deg/s) vs Time (s)')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (deg/s)')
    ## accel
    plt.subplot(313)
    plt.plot(T, ddangle_series)
    plt.title('Joint Acceleration (deg/s^2) vs Time (s)')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (deg/s^2)')
    ##
    plt.tight_layout()
    plt.show()