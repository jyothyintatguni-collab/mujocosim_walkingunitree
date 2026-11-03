import mujoco

model = mujoco.MjModel.from_xml_path("scene.xml")

print("=== ACTUATOR TYPES ===")

for i in range(model.nu):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        i
    )

    print(name)
    print("Gain type:", model.actuator_gaintype[i])
    print("Bias type:", model.actuator_biastype[i])
    print("Control limited:", model.actuator_ctrllimited[i])