import matplotlib.pyplot as plt
import numpy as np


def main():
    # stationary = open("stationary_gngga.yaml")
    # walking = open("rover_walking_gngga.yaml")
    walking = open("walking_occlusion.yaml")
    stationary = open("stationary_occlusion.yaml")

    raw_latitude = []
    raw_longitude = []
    raw_altitude = []
    raw_utm_easting = []
    raw_utm_northing = []

    for line in walking:
        line = line.strip()
        if "latitude" in line:
            raw_latitude.append(line)
        elif "longitude" in line:
            raw_longitude.append(line)
        elif "altitude" in line:
            raw_altitude.append(line)
        elif "utm_easting" in line:
            raw_utm_easting.append(line)
        elif "utm_northing" in line:
            raw_utm_northing.append(line)
        else:
            continue

    # print(raw_latitude)
    # print(raw_longitude)
    # print(raw_altitude)
    # print(raw_utm_easting)
    # print(raw_utm_northing)

    latitude = []
    for element in raw_latitude:
        result = ""
        for char in element:
            if char.isdigit():
                result += char
            elif char == ".":
                result += char
            elif char == "-":
                result += char
            else:
                continue
        latitude.append(float(result))

    longitude = []
    for element in raw_longitude:
        result = ""
        for char in element:
            if char.isdigit():
                result += char
            elif char == ".":
                result += char
            elif char == "-":
                result += char
            else:
                continue
        longitude.append(float(result))

    altitude = []
    for element in raw_altitude:
        result = ""
        for char in element:
            if char.isdigit():
                result += char
            elif char == ".":
                result += char
            elif char == "-":
                result += char
            else:
                continue
        altitude.append(float(result))

    utm_easting = []
    for element in raw_utm_easting:
        result = ""
        for char in element:
            if char.isdigit():
                result += char
            elif char == ".":
                result += char
            elif char == "-":
                result += char
            else:
                continue
        utm_easting.append(float(result))

    utm_northing = []
    for element in raw_utm_northing:
        result = ""
        for char in element:
            if char.isdigit():
                result += char
            elif char == ".":
                result += char
            elif char == "-":
                result += char
            else:
                continue
        utm_northing.append(float(result))

    print("latitude: ", latitude)
    print("longitude: ", longitude)
    print("altitude: ", altitude)
    print("utm_easting: ", utm_easting)
    print("utm_northing: ", utm_northing)

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter3D(utm_easting,utm_northing,altitude,cmap='Greens')
    plt.plot(utm_easting, utm_northing)
    # plt.xlabel("UTM_easting")
    # plt.ylabel("UTM_northing")
    plt.title("Walking 3D View")
    plt.show()




if __name__ == "__main__":
    main()