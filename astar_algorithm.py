import pdr
import numpy as np
import json
import pyastar2d
from flask import Flask, render_template,request,jsonify

topography = pdr.read("megt90n000eb.lbl")
coords = np.array(topography['IMAGE'])
dy, dx = np.gradient(coords)
infinity = 10000000
mars_r = 3389500
l = 2*3.14*mars_r
s = (l / 360) * 0.0625

dz = np.sqrt(dx**2 + dy**2)
weights = np.where(np.degrees(np.arctan(dz/s)) < 20, 1 + dz, infinity)

weights = np.array(weights,dtype=np.float32)
safety_zones = [[49.02, -171.85],[-42.4,70.5]]
def transform_from_latlng(lat,lng):
    step = 0.0625
    if (lng < 0):
        x = 180 / step + (180 + lng) / step
    else:
        x = lng / step
    y = (90 - lat) / step
    return round(y),round(x)

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
    mn_distance = safety_zones[0]
    mn = 10000000
    data_e = request.json
    lat = data_e['lat']
    lng = data_e['lng']
    for i in range(len(safety_zones)):
        if ((abs(lat - safety_zones[i][0]) + abs(lng - safety_zones[i][1])) < mn):
            mn = abs(lat - safety_zones[i][0]) + abs(lng - safety_zones[i][1])
            mn_distance = safety_zones[i]
    path = pyastar2d.astar_path(weights, transform_from_latlng(lat,lng),transform_from_latlng(mn_distance[0],mn_distance[1]),allow_diagonal=True)
    final = transform_from_yx(path)

    with open("heights.json","w") as f:
        json.dump([final.tolist()],f)

    return jsonify([final.tolist()])


if __name__ == "__main__":
    app.run(debug = True,port = 5000)
