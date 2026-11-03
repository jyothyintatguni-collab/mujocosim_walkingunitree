
import time
import numpy as np
import mujoco
import mujoco.viewer

# Load Go1 model
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

# Initialize simulation
mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)

# Map actuator names to joint positions
actuator_info = {}

for i in range(model.nu):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        i
    )

    joint_id = model.actuator_trnid[i, 0]
    qpos_id = model.jnt_qposadr[joint_id]

    actuator_info[name] = (i, qpos_id)

# Initial joint positions
home = data.qpos.copy()

# Diagonal gait phase
phase_offset = {
    "FR": 0.0,
    "RL": 0.0,
    "FL": np.pi,
    "RR": np.pi
}

# Gait parameters (initial experimental values)
frequency = 1.0
amplitude = {
    "hip": 0.15,
    "thigh": 0.12,
    "calf": 0.20
}

# Hold initial position
hold_time = 2.0
start_time = time.time()

print("Starting Go1 simulation...")

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        current_time = time.time() - start_time

        # Generate gait after initial hold
        if current_time < hold_time:
            for name, (act_id, qpos_id) in actuator_info.items():
                data.ctrl[act_id] = home[qpos_id]

        else:
            t = current_time - hold_time

            for leg, offset in phase_offset.items():

                phase = 2 * np.pi * frequency * t + offset

                for joint in ["hip", "thigh", "calf"]:

                    name = f"{leg}_{joint}"

                    act_id, qpos_id = actuator_info[name]

                    base = home[qpos_id]

                    if joint == "hip":
                        movement = amplitude["hip"] * np.sin(phase)

                    elif joint == "thigh":
                        movement = amplitude["thigh"] * np.sin(
                            phase + np.pi / 2
                        )

                    else:
                        movement = amplitude["calf"] * np.sin(
                            phase - np.pi / 2
                        )

                    target = base + movement

                    # Respect actuator control limits
                    if model.actuator_ctrllimited[act_id]:
                        lower, upper = model.actuator_ctrlrange[act_id]
                        target = np.clip(target, lower, upper)

                    data.ctrl[act_id] = target

        # Advance physics
        mujoco.mj_step(model, data)

        viewer.sync()

        # Maintain approximately real-time simulation
        time.sleep(model.opt.timestep)

print("Simulation stopped.")
