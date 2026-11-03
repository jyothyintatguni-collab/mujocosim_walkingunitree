import mujoco

model = mujoco.MjModel.from_xml_path("scene.xml")

print("\n=== MOTOR CONFIGURATION ===")

for i in range(model.nu):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        i
    )

    print(f"\nMotor: {name}")
    print("Control range:", model.actuator_ctrlrange[i])
    print("Force range:", model.actuator_forcerange[i])