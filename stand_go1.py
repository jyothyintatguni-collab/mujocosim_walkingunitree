
import time
import mujoco
import mujoco.viewer

# Load the Unitree Go1 model
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

# Find the predefined standing pose
home_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home"
)

if home_id < 0:
    raise ValueError("Home keyframe not found!")

# Initialize robot in standing pose
mujoco.mj_resetDataKeyframe(model, data, home_id)

print("Go1 standing controller started!")

# Launch simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)
