import time
import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

# set the simulator into stepping mode - useful when using external scripts like this one.
client.setStepping(True)

try:
    # get a handle for each joint
    # <-- enter code here -->
    joint_count = 6
    joint_handle = [sim.getObject(f"/Joint{x+1}") for x in range(joint_count)]
    
    # graphing
    graph_handle = sim.getObject('/Graph')
    tool_handle = sim.getObject('/ToolTip')
    floor_handle = sim.getObject('/Floor')
    tx_id = sim.addGraphStream(graph_handle, '/tool_x', '', 0, [1,0,0],0)
    ty_id = sim.addGraphStream(graph_handle, '/tool_y', '', 0, [0,1,0],0)
    tz_id = sim.addGraphStream(graph_handle, '/tool_z', '', 0, [0,0,1],0)
    sim.addGraphCurve(graph_handle, 'tool_pos', 3, [tx_id, ty_id, tz_id], [0, 0, 0], '', 0,
                      [0,1,0], 2)
    print("simulation is starting")
    sim.startSimulation()

    # set the joint angle
    # <-- enter code here -->
    q = [10, 20, 30, 40, 50, 60]
    qd = 5 # deg per second
    [sim.setJointTargetPosition(joint_handle[i], q[i]*math.pi/180) for i in range(joint_count)]

    # Use this loop to visualise motion in smaller increments
    prev_time = sim.getSimulationTime()
    while (t := sim.getSimulationTime()) < 3:
        dt = t - prev_time
        s = f'Simulation time: {t:.2f} [s]'
        print(s)
        q = [joint_pos + qd*dt for joint_pos in q]
        [sim.setJointPosition(joint_handle[i], q[i]*math.pi/180) for i in range(joint_count)]
        client.step()
        tool_x, tool_y, tool_z = sim.getObjectPosition(tool_handle, sim.handle_world)
        sim.setGraphStreamValue(graph_handle, tx_id, tool_x)
        sim.setGraphStreamValue(graph_handle, ty_id, tool_y)
        sim.setGraphStreamValue(graph_handle, tz_id, tool_z)
        prev_time = t

    # pause the simulation
    sim.pauseSimulation()
    print("simulation is paused")

    # stop the simulation
    # sim.stopSimulation()
    # print("simulation is done")
except KeyboardInterrupt:
    sim.pauseSimulation()
    sim.stopSimulation()