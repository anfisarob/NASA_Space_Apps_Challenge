import pdr
import numpy as np
import json
import pyastar2d
from flask import Flask, render_template,request,jsonify
import math
from PIL import Image

img = Image.open("./src/thermal_inertia.png").convert("HSV")

arr = np.array(img)
new_arr = np.repeat(arr,2,axis = 1)
new_arr = np.repeat(new_arr,2,axis = 0)

value_arr = new_arr[:,:,0]
intensity = new_arr[:,:,2]
saturation = new_arr[:,:,1]

mn = 25
mx = 600

value_arr = np.where((intensity < 15) | (saturation < 15), np.average(value_arr), value_arr)
thermal_inertia = mn + (mx - mn)*((value_arr - value_arr.min()) / (value_arr.max() - value_arr.min()))


topography = pdr.read("megt90n000eb.lbl")
coords = np.array(topography['IMAGE'])
dy, dx = np.gradient(coords)
infinity = np.inf
mars_r = 3396000
l = 2*math.pi*mars_r
s = (l / 360) * 0.0625

dz = np.sqrt(dx**2 + dy**2)
weights = np.where(np.degrees(np.arctan(dz/s)) < 20, 1 + dz, infinity)

thermal_k = 0.1
weights = np.array(weights,dtype=np.float32) + np.array(thermal_k*thermal_inertia,dtype=np.float32)
safety_zones = [[49.02, -171.85],[-42.4,70.5]]

def transform_from_latlng(lat,lng):
    step = 0.0625
    if (lng < 0):
        x = 180 / step + (180 + lng) / step
    else:
        x = lng / step
    y = (90 - lat) / step

    y = min(max(round(y),0),coords.shape[0]-1)
    x = min(max(round(x),0),coords.shape[1]-1)

    return y,x

def transform_from_yx(path):
    step = 0.0625
    lng0 = step * path[:,1]
    lng = np.where(lng0 > 180, step * path[:,1] - 360, step * path[:,1])
    lat = 90 - step * path[:,0]
    return np.column_stack((lat, lng))

app = Flask(__name__,static_folder='.',static_url_path = '')

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/api/build_path", methods = ["POST"])
def build_path():
    data = request.json

    start_x, start_y = transform_from_latlng(data['lat'], data['lng'])

    goal_x = -4.5895
    goal_y = 137.4417

    path = pyastar2d.astar_path(weights, (start_x, start_y), transform_from_latlng(goal_x,goal_y), allow_diagonal=True)
    final = transform_from_yx(path)

    with open("heights.json", "w") as f:
        json.dump([final.tolist()],f)

    return jsonify([final.tolist()])

@app.route("/api/build_path_emergency",methods=['POST'])
def build_path_emergency():
    best_cost = infinity
    best_path = None
    data_e = request.json
    lat = data_e['lat']
    lng = data_e['lng']

    for zone in safety_zones:
        path = pyastar2d.astar_path(weights,transform_from_latlng(lat,lng),transform_from_latlng(zone[0],zone[1]))

        cost = weights[path[:,0],path[:,1]].sum()

        if (cost < best_cost):
            best_cost = cost
            best_path = path

    final = transform_from_yx(best_path)

    with open("heights.json","w") as f:
        json.dump([final.tolist()],f)

    return jsonify([final.tolist()])


if __name__ == "__main__":
    app.run(debug = True,port = 5000)
