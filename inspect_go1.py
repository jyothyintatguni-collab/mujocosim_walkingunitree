import mujoco

model = mujoco.MjModel.from_xml_path("scene.xml")

print("\n=== GO1 JOINTS ===")

for i in range(model.njnt):
    print(i, mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_JOINT, i
    ))

print("\nTotal joints:", model.njnt)
print("Total actuators:", model.nu)

print("\n=== ACTUATORS ===")

for i in range(model.nu):
    print(i, mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, i
    ))