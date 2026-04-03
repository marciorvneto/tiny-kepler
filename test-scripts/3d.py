import numpy as np

__mu = 398600  # km³/s²


def rot_z(theta):
    return np.array(
        [
            [np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1],
        ]
    )


def rot_y(theta):
    return np.array(
        [
            [np.cos(theta), 0, -np.sin(theta)],
            [0, 1, 0],
            [np.sin(theta), 0, np.cos(theta)],
        ]
    )


def rot_x(theta):
    return np.array(
        [
            [
                1,
                0,
                0,
            ],
            [
                0,
                np.cos(theta),
                np.sin(theta),
            ],
            [0, -np.sin(theta), np.cos(theta)],
        ]
    )


def apply_rot(rot, angle, r):
    raw = rot(angle) @ r
    raw[np.abs(raw) < 1e-12] = 0
    return raw


def test_rots():
    i = np.array([1, 0, 0])
    j = np.array([0, 1, 0])
    k = np.array([0, 0, 1])

    Rx_i = apply_rot(rot_x, np.pi / 2, i)
    Rx_j = apply_rot(rot_x, np.pi / 2, j)
    Rx_k = apply_rot(rot_x, np.pi / 2, k)

    Ry_i = apply_rot(rot_y, np.pi / 2, i)
    Ry_j = apply_rot(rot_y, np.pi / 2, j)
    Ry_k = apply_rot(rot_y, np.pi / 2, k)

    Rz_i = apply_rot(rot_z, np.pi / 2, i)
    Rz_j = apply_rot(rot_z, np.pi / 2, j)
    Rz_k = apply_rot(rot_z, np.pi / 2, k)

    print("====== Rx ======")
    print("i:")
    print(Rx_i)
    print("j:")
    print(Rx_j)
    print("k:")
    print(Rx_k)

    print("====== Ry ======")
    print("i:")
    print(Ry_i)
    print("j:")
    print(Ry_j)
    print("k:")
    print(Ry_k)

    print("====== Rz ======")
    print("i:")
    print(Rz_i)
    print("j:")
    print(Rz_j)
    print("k:")
    print(Rz_k)


def elements_to_state(h, e, theta, ra, w, i):
    # r = h²/mu * 1 / (1 + e cos(theta))
    r_norm = h * h / __mu * 1.0 / (1.0 + e * np.cos(theta))
    rp = np.array([r_norm * np.cos(theta), r_norm * np.sin(theta), 0])
    Q = rot_z(w) @ rot_x(i) @ rot_z(ra)

    # vr = mu/h * e * sin(theta)
    # vt = mu/h * (1 + e * cos(thea))

    z = np.array([0, 0, 1])
    ur = rp / r_norm
    ut = np.cross(z, ur)
    vp = __mu / h * (ut * (1 + e * np.cos(theta)) + ur * e * np.sin(theta))

    r = Q.T @ rp
    v = Q.T @ vp
    return r, v


def state_to_elements(r, v):
    I = np.array([1, 0, 0])
    K = np.array([0, 0, 1])
    r_norm = np.linalg.norm(r)
    ur = r / r_norm
    vr = np.dot(v, ur)
    h = np.cross(r, v)
    h_norm = np.linalg.norm(h)
    uh = h / h_norm
    i = np.arccos(np.dot(K, uh))

    # ra

    N = np.cross(K, uh)
    un = N / np.linalg.norm(N)
    N_dot_I = np.dot(un, I)
    ra = np.arccos(N_dot_I) if N[1] > 0 else 2 * np.pi - np.arccos(N_dot_I)

    # Eccentricity

    E = np.cross(v, h) / __mu - r / r_norm

    e = np.linalg.norm(E)
    uE = E / e

    # Real anomaly

    ur_dot_uE = ur.dot(uE)
    theta = np.arccos(ur_dot_uE) if vr >= 0 else 2 * np.pi - np.arccos(ur_dot_uE)

    un_dot_uE = un.dot(uE)
    w = np.arccos(un_dot_uE) if E[2] >= 0 else 2 * np.pi - np.arccos(un_dot_uE)

    print(f"h_norm = {h_norm:.2f}")
    print(f"e = {e:.2f}")
    print(f"i = {i:0.2f} = {i * 180 / np.pi:0.2f} deg")
    print(f"ra = {ra:0.2f} = {ra * 180 / np.pi:0.2f} deg")
    print(f"theta = {theta:0.2f} = {theta * 180 / np.pi:0.2f} deg")
    print(f"w = {w:0.2f} = {w * 180 / np.pi:0.2f} deg")

    # r = h²/mu * 1 / (1 + e cos(theta))

    # epsilon = -mu/2a


# 1
# state_to_elements(np.array([2615, 15881, 3980]), np.array([-2.767, -0.7905, 4.980]))
# 2
state_to_elements(np.array([0, 0, 12670]), np.array([0, -3.874, -0.7905]))

# test_rots()

r, v = elements_to_state(
    h=80_000, e=1.4, theta=np.pi / 6, ra=40.0 * np.pi / 180, w=np.pi / 3, i=np.pi / 6
)

# import matplotlib.pyplot as plt

# thetas = np.linspace(0, 2 * np.pi, 500)
# positions = np.array(
#     [
#         elements_to_state(
#             h=80000, e=0.6, theta=t, ra=40 * np.pi / 180, w=np.pi / 3, i=np.pi / 6
#         )[0]
#         for t in thetas
#     ]
# )

# positions_2 = np.array(
#     [
#         elements_to_state(
#             h=80000, e=0.6, theta=t, ra=40 * np.pi / 180, w=np.pi / 3, i=np.pi / 2
#         )[0]
#         for t in thetas
#     ]
# )

# fig = plt.figure(figsize=(10, 10))
# ax = fig.add_subplot(111, projection="3d")
# ax.plot(positions[:, 0], positions[:, 1], positions[:, 2])
# ax.plot(positions_2[:, 0], positions_2[:, 1], positions_2[:, 2], "r")
# ax.plot([0], [0], [0], "yo", markersize=10)
# ax.set_xlabel("X (km)")
# ax.set_ylabel("Y (km)")
# ax.set_zlabel("Z (km)")
# ax.set_aspect("equal")


# # After creating the axis, before plt.show()

# # Draw coordinate axes
# lim = np.max(np.abs(positions)) * 0.5
# ax.plot([-lim, lim], [0, 0], [0, 0], "r-", alpha=0.3, linewidth=0.5)
# ax.plot([0, 0], [-lim, lim], [0, 0], "g-", alpha=0.3, linewidth=0.5)
# ax.plot([0, 0], [0, 0], [-lim, lim], "b-", alpha=0.3, linewidth=0.5)

# # Draw equatorial plane (xy)
# xx, yy = np.meshgrid(np.linspace(-lim, lim, 2), np.linspace(-lim, lim, 2))
# zz = np.zeros_like(xx)
# ax.plot_surface(xx, yy, zz, alpha=0.05, color="gray")

# plt.show()
