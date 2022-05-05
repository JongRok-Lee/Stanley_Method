from cmath import inf
import numpy as np
import matplotlib.pyplot as plt

# paramters
dt = 0.1
k = 2  # control gain

# GV70 PARAMETERS
LENGTH, WIDTH = 4.715, 1.910
L = 2.875
BACKTOWHEEL = 1.0
WHEEL_LEN, WHEEL_WIDTH = 0.3, 0.2  # [m]
TREAD = 0.8  # [m]
max_steering = np.radians(30) # max_angle = 30 degree

def make_ref(road : str ="linear"):
    
    if road == "linear":
        ref_xs = np.linspace(0, 500, 500)
        ref_ys = np.zeros_like(ref_xs) + 1.5
        ref_yaws = np.arctan(np.gradient(ref_ys))
    elif road == 'sin':
        ref_xs = np.linspace(0, 500, 500)
        ref_ys = 5 * np.sin(0.2*ref_xs) + 1
        ref_yaws = np.arctan(np.gradient(ref_ys))
    elif road == 'circle':
        theta = np.linspace(0, 2*np.pi, 500)
        ref_xs = 20 * np.cos(theta)-20
        ref_ys = 20 * np.sin(theta)
        ref_yaws = np.pi/2 + theta
    
    return ref_xs, ref_ys, ref_yaws
        



class VehicleModel(object):
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw*np.pi/180
        self.v = v

    def update(self, steer):
        self.yaw += self.v / L * np.tan(steer) * dt         # yaw(n+1)    = yaw(n) + L * v * tan(steer) * dt
        self.yaw = self.yaw % (2.0 * np.pi)                 # yaw = yaw % 2π
        self.x += self.v * np.cos(self.yaw) * dt            # x(n+1)      = x(n) + v * cos(yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt            # y(n+1)      = y(n) * v * sin(yaw) * dt
        self.v = self.v                                     # v(n+1) = v(n)


def normalize_angle(angle):
    while angle > np.pi:        # angle > π  ----> angle = angle - 2π
        angle -= 2.0 * np.pi    # ex) 270 degree = -90 degree

    while angle < -np.pi:       # angle < π  ----> angle = angle + 2π
        angle += 2.0 * np.pi    # ex) -270 degree = 90 degree

    return angle


def stanley_control(x, y, yaw, v, ref_xs, ref_ys, ref_yaws):
    # find nearest point
    min_dist = 1e9
    min_index = 0
    n_points = len(ref_xs)

    front_x = x + L * np.cos(yaw) # x = back wheel position
    front_y = y + L * np.sin(yaw) # y = back wheel position

    for i in range(n_points):     # find x coordinate index
        dx = front_x - ref_xs[i]
        dy = front_y - ref_ys[i]

        dist = np.sqrt(dx * dx + dy * dy)
        if dist < min_dist:
            min_dist = dist
            min_index = i

    # compute cte at front axle
    ref_x = ref_xs[min_index]
    ref_y = ref_ys[min_index]
    ref_yaw = ref_yaws[min_index]
    dx = ref_x - front_x
    dy = ref_y - front_y
    
    perp_vec = [np.cos(ref_yaw + np.pi/2), np.sin(ref_yaw + np.pi/2)]
    cte1 = np.dot([dx, dy], perp_vec)
    cte2 = np.sqrt(np.dot([dx, dy],[dx, dy]))
    if ref_y < front_y:
        cte2 *= -1

    # control law
    psi = normalize_angle(ref_yaw - yaw)
    cte_term = np.arctan2(k*cte1, v)

    # steering
    steer = psi + cte_term
    steer = np.clip(steer, -max_steering, max_steering) # limit the steering angle
    return steer, ref_x, ref_y



def plot(xs, ys, yaws, steers, ref_xs, ref_ys, dxs, dys):
    plt.figure(figsize=(12, 3))
    plt.plot(ref_xs[:60], ref_ys[:60], 'r--', alpha=0.5, label="reference")
    for i in range(len(xs)):
        # plt.clf()
        if i % 20 == 0:
            
            plt.plot(xs, ys, 'b--', alpha=0.5)
            x = xs[i]
            y = ys[i]
            yaw = yaws[i]
            steer = steers[i]
            dx = dxs[i]
            dy = dys[i]

            outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                                [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
            fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                                [-WHEEL_WIDTH, -WHEEL_WIDTH, WHEEL_WIDTH, WHEEL_WIDTH, -WHEEL_WIDTH]])

            rr_wheel = np.copy(fr_wheel)
            fl_wheel = np.copy(fr_wheel)
            rl_wheel = np.copy(rr_wheel)

            Rot1 = np.array([[np.cos(yaw), np.sin(yaw)],
                            [-np.sin(yaw), np.cos(yaw)]])
            Rot2 = np.array([[np.cos(steer+yaw), np.sin(steer+yaw)],
                            [-np.sin(steer+yaw), np.cos(steer+yaw)]])

            fr_wheel = (fr_wheel.T.dot(Rot2)).T
            fl_wheel = (fl_wheel.T.dot(Rot2)).T
            fr_wheel[0, :] += L * np.cos(yaw) - TREAD * np.sin(yaw)
            fl_wheel[0, :] += L * np.cos(yaw) + TREAD * np.sin(yaw)
            fr_wheel[1, :] += L * np.sin(yaw) + TREAD * np.cos(yaw)
            fl_wheel[1, :] += L * np.sin(yaw) - TREAD * np.cos(yaw)
            rr_wheel[1, :] += TREAD
            rl_wheel[1, :] -= TREAD

            outline = (outline.T.dot(Rot1)).T
            rr_wheel = (rr_wheel.T.dot(Rot1)).T
            rl_wheel = (rl_wheel.T.dot(Rot1)).T

            outline[0, :] += x
            outline[1, :] += y
            fr_wheel[0, :] += x
            fr_wheel[1, :] += y
            rr_wheel[0, :] += x
            rr_wheel[1, :] += y
            fl_wheel[0, :] += x
            fl_wheel[1, :] += y
            rl_wheel[0, :] += x
            rl_wheel[1, :] += y

            plt.plot(np.array(outline[0, :]).flatten(),
                    np.array(outline[1, :]).flatten(), 'k-', alpha=0.5)
            plt.plot(np.array(fr_wheel[0, :]).flatten(),
                    np.array(fr_wheel[1, :]).flatten(), 'k-')
            plt.plot(np.array(rr_wheel[0, :]).flatten(),
                    np.array(rr_wheel[1, :]).flatten(), 'k-')
            plt.plot(np.array(fl_wheel[0, :]).flatten(),
                    np.array(fl_wheel[1, :]).flatten(), 'k-')
            plt.plot(np.array(rl_wheel[0, :]).flatten(),
                    np.array(rl_wheel[1, :]).flatten(), 'k-')
            # plt.plot(x, y, "bo")
            plt.plot(dx, dy, "ro")
            plt.plot(x + L *np.cos(yaw), y + L *np.sin(yaw), "go")
            plt.axis("equal")
            plt.pause(0.2)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig("stanley_method.png", dpi=300)
    
    plt.show()