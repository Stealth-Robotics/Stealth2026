import json
import os

field_width = 57.573 / 3.281
field_height = 26.417 / 3.281

choreoDirectory = os.path.join(os.path.dirname(__file__), "deploy/choreo")


def load_traj(file_path):
    with open(file_path, 'r') as f:
        traj = json.load(f)
    return traj

def mirror_snap_point(point):
    #Mirror a point across the y centerline of the field making sure to rotate accordingly
    point = {
        "x": point["x"],
        "y": field_height - point["y"],
        "heading": -point["heading"],
        "intervals": point["intervals"],
        "split": point["split"],
        "fixTranslation": point["fixTranslation"],
        "fixHeading": point["fixHeading"],
        "overrideIntervals": point["overrideIntervals"]
    }
    return point

def flipExpression(expression):
    exp = expression["exp"]
    val = expression["val"]

    #add - to the exp value if it is not already there
    if exp[0] != "-":
        exp = "-" + exp
    else:
        exp = exp[1:]

    val = -val

    return {
        "exp": exp,
        "val": val
    }

def mirror_param_point(point):
    pYVal = field_height - point["y"]["val"]
    pYExp = str(pYVal) + " m"
    point["y"]["val"] = pYVal
    point["y"]["exp"] = pYExp

    pHeadingVal = -point["heading"]["val"]
    pHeadingExp = str(pHeadingVal) + " rad"

    point["heading"]["val"] = pHeadingVal
    point["heading"]["exp"] = pHeadingExp

    return point

def main():
    choreoFiles = os.listdir(choreoDirectory)

    #Find only .traj files that do not contain the word "mirrored"
    trajFiles = [file for file in choreoFiles if file.endswith(".traj") and file.startswith("m_")]

    for file in trajFiles:
        traj = load_traj(choreoDirectory + "/" + file)
        snapshot_waypoints = traj["snapshot"]["waypoints"]
        param_waypoints = traj["params"]["waypoints"]
        
        for i, point in enumerate(snapshot_waypoints):
            snapshot_waypoints[i] = mirror_snap_point(point)

        for i, point in enumerate(param_waypoints):
            param_waypoints[i] = mirror_param_point(point)

        # Determine new filename (flip Left/Right)
        base_name = file[2:]  # remove "m_"

        if base_name.startswith("Left"):
            new_name = "Right" + base_name[len("Left"):]
        elif base_name.startswith("Right"):
            new_name = "Left" + base_name[len("Right"):]
        else:
            # fallback if neither left nor right is in the name
            new_name = base_name

        # save the mirrored traj
        with open(os.path.join(choreoDirectory, new_name), 'w') as f:
            json.dump(traj, f, indent=4)

        original_path = os.path.join(choreoDirectory, file)
        renamed_path = os.path.join(choreoDirectory, base_name)

        os.rename(original_path, renamed_path)

        print("successfully mirrored " + file + " -> " + new_name)

if __name__ == '__main__':
    main()