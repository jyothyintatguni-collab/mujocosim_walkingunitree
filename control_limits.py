
import mujoco

model = mujoco.MjModel.from_xml_path("scene.xml")

print("=== ACTUATOR CONTROL LIMITS ===")

for i in range(model.nu):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        i
    )

    print(f"\nMotor: {name}")
    print("Control limited:", model.actuator_ctrllimited[i])
    print("Control range:", model.actuator_ctrlrange[i])
