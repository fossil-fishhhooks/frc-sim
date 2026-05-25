import json

base = {
    "name": "test_floor_robot",
    "bodies": [
        {
            "def": "../bodies/floor.json",
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "role": "field"
        },
        {
            "def": "../bodies/robot.json",
            "position": [5.0, 0.1, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "role": "robot"
        }
    ]
}

spacing = 0.2
nx = 10
nz = 40

for i in range(nx):
    for j in range(nz):
        x = (i - (nx - 1) / 2) * spacing
        z = (j - (nz - 1) / 2) * spacing

        base["bodies"].append({
            "def": "../bodies/sphere.json",
            "position": [x, 2.0, z],
            "orientation": [0, 0, 0, 1],
            "role": "gamepiece"
        })

print(json.dumps(base, indent=2))
