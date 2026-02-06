import pinocchio as pin
import numpy as np
print(dir(pin))

# Load your URDF
model = pin.buildModelFromUrdf("scaled_ur.urdf")
data = model.createData()

# Compute gravity torque
q = np.zeros(6)  # joint angles
tau_gravity = pin.computeGeneralizedGravity(model, data, q)
print("done")