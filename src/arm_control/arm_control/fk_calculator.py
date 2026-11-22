import math

# Parameter dimensi dari arm.urdf (meter)
L0 = 0.10  # base_link -> joint_1
L1 = 0.25  # joint_1 -> joint_2 (link_2)
L2 = 0.20  # joint_2 -> joint_3 (link_3)

def forward_kinematics(theta1_rad, theta2_rad):
    r = L2 * math.sin(theta2_rad)
    x = r * math.cos(theta1_rad)
    y = r * math.sin(theta1_rad)
    z = L0 + L1 + L2 * math.cos(theta2_rad)
    return x, y, z

if __name__ == '__main__':
    q1 = 0.8
    q2 = 0.7
    x, y, z = forward_kinematics(q1, q2)
    print(f"Input  : joint_1 = {q1} rad, joint_2 = {q2} rad")
    print(f"Output : X = {x:.7f} m, Y = {y:.7f} m, Z = {z:.7f} m")