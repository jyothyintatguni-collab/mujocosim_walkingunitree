import mujoco

model = mujoco.MjModel.from_xml_path("scene.xml")

print("\n=== ACTUATOR DETAILS ===")

for i in range(model.nu):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        i
    )

    print(f"\nMotor: {name}")
    print("Gear:", model.actuator_gear[i])
    print("Gain parameters:", model.actuator_gainprm[i][:3])
    print("Bias parameters:", model.actuator_biasprm[i][:3])
    print("Dynamics type:", model.actuator_dyntype[i])