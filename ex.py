import numpy as np
from tools import *

# map
ref_xs, ref_ys, ref_yaws = make_ref(road="circle")

# vehicle
model = VehicleModel(x=0.0, y=0.0, yaw=0, v=1.0)
steer = 0


xs, ys, yaws = [[], [], []]
steers = []
ts = []
dxs, dys = [], []
for step in range(600):
    t = step * dt
    steer, dx, dy = stanley_control(model.x, model.y, model.yaw, model.v, ref_xs, ref_ys, ref_yaws)

    xs.append(model.x)
    ys.append(model.y)
    yaws.append(model.yaw)
    ts.append(t)
    steers.append(steer)
    dxs.append(dx)
    dys.append(dy)
    
    model.update(steer)

# plot car
plot(xs, ys, yaws, steers, ref_xs, ref_ys, dxs, dys)