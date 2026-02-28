# Importing the necessary libraries
import figs.render.capture_generation as pg
import figs.visualize.plot_trajectories as pt
import figs.visualize.generate_videos as gv
import figs.scene_editing.scene_editing_utils as scdt

from figs.simulator import Simulator
from figs.control.vehicle_rate_mpc import VehicleRateMPC

import os
import numpy as np

#%%
# FiGS Capture Examples (scene_name, capture_name)
capture_examples = [
    'flightroom_ssv_exp'
]

# FiGS Simulate Examples (scene_name, rollout_name, frame_name, policy_name, course_name)
# NOTE: scene_name will be updated after Step F1 completes with the actual splatfacto timestamp path
simulate_examples = [
    ('flightroom_ssv_exp/splatfacto/2026-02-23_015136',   'baseline', 'carl', 'vrmpc_rrt', 'track_spiral')
]

# query = 'ladder'

#%%

# Simulate within the FiGS environment
for scene, rollout, frame, policy, course in simulate_examples:
    print("=============================================================")
    print(f"Simulating {scene} scene with {course} course")
    print("-------------------------------------------------------------")

    # Load the policy and simulator
    sim = Simulator(scene,rollout,frame)
    ctl = VehicleRateMPC(course,policy,frame)

    # Use the ideal trajectory in VehicleRateMPC to get initial conditions and final time
    t0,tf,x0 = ctl.tXUd[0,0],ctl.tXUd[0,-1],ctl.tXUd[1:11,0]

    # Simulate the policy
    Tro,Xro,Uro,Imgs,_,_ = sim.simulate(ctl,t0,tf,x0)

    # Output the results
    gv.images_to_mp4(Imgs["rgb"],'test_space/'+course+'_'+scene+'.mp4', ctl.hz)       # Save the video
    # pt.plot_RO_spatial((Tro,Xro,Uro))                               # Plot the spatial trajectory

    # scdt.plot_point_cloud(sim,
    #                       (Tro,Xro,Uro),
    #                       50)

    # Clear the memory of the policy to avoid improper re-initialization of ACADOS
    del ctl