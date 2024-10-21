import time

import numpy as np
import sapien.core as sapien
from sapien.utils.viewer import Viewer


def demo(fix_root_link, balance_passive_force):
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    scene_config = sapien.SceneConfig()
    scene = engine.create_scene(scene_config)
    scene.set_timestep(1 / 500.0)
    scene.add_ground(0)

    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=-2, y=0, z=1)
    viewer.set_camera_rpy(r=0, p=-0.3, y=0)

    # Load URDF
    loader: sapien.URDFLoader = scene.create_urdf_loader()
    loader.fix_root_link = fix_root_link
    loader.load_multiple_collisions_from_file = True

    robot: sapien.Articulation = loader.load(
        "ManiSkill2_real2sim/mani_skill2_real2sim/assets/descriptions/panda_robotiq_85_alt_2.urdf"
    )
    
    print(robot.get_links())
    robot.set_root_pose(sapien.Pose([0, 0, 0.06205], [1, 0, 0, 0]))

    # Set initial joint positions
    # qpos = [
    #     -0.2639457174606611,
    #     0.0831913360274175,
    #     0.5017611504652179,
    #     1.156859026208673,
    #     0.028583671314766423,
    #     1.592598203487462,
    #     -1.080652960128774,
    #     0,
    #     0,
    #     -0.00285961,
    #     0.7851361,
    # ]
    qpos = [0.0, 0.003, -0.002, -0.944, 0.019, 1.195, 0.005, -0.021, -0.00, -0.00, -0.022, 0.001, 0.017]
    robot.set_qpos(qpos)
    for joint in robot.get_active_joints():
        joint.set_drive_property(stiffness=1e5, damping=1e3)

    while not viewer.closed:
        # print(robot.get_qpos())
        for _ in range(4):  # render every 4 steps
            if balance_passive_force:
                qf = robot.compute_passive_force(
                    gravity=True,
                    coriolis_and_centrifugal=True,
                )
                robot.set_qf(qf)
            print("target qpos", qpos)
            print("current qpos", robot.get_qpos())
            # robot.set_drive_target(qpos)
            scene.step()
        scene.update_render()
        viewer.render()


def main():
    demo(fix_root_link=True, balance_passive_force=True)


if __name__ == "__main__":
    main()
    """
    robot.qpos 13-dim if mobile else 11
    robot qlimits
        array([[     -inf,       inf],
       [     -inf,       inf],
       [-4.49e+00,  1.35e+00],
       [-2.66e+00,  3.18e+00],
       [-2.13e+00,  3.71e+00],
       [-2.05e+00,  3.79e+00],
       [-2.92e+00,  2.92e+00],
       [-1.79e+00,  1.79e+00],
       [-4.49e+00,  1.35e+00],
       [-1.00e-04,  1.30e+00], # gripper plus direction = close
       [-1.00e-04,  1.30e+00],
       [-3.79e+00,  2.22e+00],
       [-1.17e+00,  1.17e+00]], dtype=float32)
    robot.get_active_joints()
        ['joint_wheel_left', 'joint_wheel_right', 'joint_torso', 'joint_shoulder',
        'joint_bicep', 'joint_elbow', 'joint_forearm', 'joint_wrist', 'joint_gripper',
        'joint_finger_right', 'joint_finger_left', 'joint_head_pan', 'joint_head_tilt']
    If robot is not mobile, then the first two joints are not active
    """
