import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":

    latitude = []
    longitude = []
    altitude = []
    utm_easting = []
    utm_northing = []

    # filename = open("stationary_data.yaml", "r")
    filename = open("walking_data.yaml", "r")
    for line in filename:
        line = line.strip()
        # print(line)
        if "#" in line:
            continue
        elif "latitude" in line:
            latitude.append(line)
        elif "longitude" in line:
            longitude.append(line)
        elif "altitude" in line:
            altitude.append(line)
        elif "utm_easting" in line:
            utm_easting.append(line)
        elif "utm_northing" in line:
            utm_northing.append(line)

    print("Displaying gathered raw data:")
    print(latitude)
    print(longitude)
    print(altitude)
    print(utm_easting)
    print(utm_northing)

    print()

    print("Sorting data to list and convert into float number without rounding")

    new_latitude = []
    for element in latitude:
        result = ""
        for char in element:
            if char.isdigit():
                result += char
            elif char == "-":
                result += char
            elif char == ".":
                result += char
            else:
                continue
        new_latitude.append(float(result))

    print("Latitude: ", new_latitude)

    new_longitude = []
    for element in longitude:
        result = ""
        for char in element:
            if char.isdigit():
                result += char
            elif char == "-":
                result += char
            elif char == ".":
                result += char
            else:
                continue
        new_longitude.append(float(result))

    print("Longitude: ", new_longitude)

    new_altitude = []
    for element in altitude:
        result = ""
        for char in element:
            if char.isdigit():
                result += char
            elif char == "-":
                result += char
            elif char == ".":
                result += char
            else:
                continue
        new_altitude.append(float(result))

    print("Altitude: ", new_altitude)

    new_utm_easting = []
    for element in utm_easting:
        result = ""
        for char in element:
            if char.isdigit():
                result += char
            elif char == "-":
                result += char
            elif char == ".":
                result += char
            else:
                continue
        new_utm_easting.append(float(result))

    print("UTM_easting: ", new_utm_easting)

    new_utm_northing = []
    for element in utm_northing:
        result = ""
        for char in element:
            if char.isdigit():
                result += char
            elif char == "-":
                result += char
            elif char == ".":
                result += char
            else:
                continue
        new_utm_northing.append(float(result))

    print("UTM_northing: ", new_utm_northing)

    # print(len(new_altitude))
    counter = []
    count = 0
    for i in range(len(new_altitude)):
        counter.append(count)
        count += 1

    # print(new_altitude.index(max(new_altitude)))
    # print(counter)

    plt.plot(new_utm_northing)
    # plt.bar(counter, new_altitude)
    # plt.annotate('max altitude: ' + str(max(new_altitude)),
    #              xy=(new_altitude.index(max(new_altitude)), max(new_altitude)), xytext=(600, 19),
    #              arrowprops=dict(facecolor='red', shrink=0.05),
    #              )
    # plt.annotate('min altitude: ' + str(min(new_altitude)),
    #              xy=(new_altitude.index(min(new_altitude)), min(new_altitude)), xytext=(0, 22),
    #              arrowprops=dict(facecolor='red', shrink=0.05),
    #              )
    # plt.annotate('max longitude: ' + str(max(new_longitude)),
    #              xy=(new_longitude.index(max(new_longitude)), max(new_longitude)), xytext=(500, -71.07360),
    #              arrowprops=dict(facecolor='red', shrink=0.05),
    #              )
    # plt.annotate('min longitude: ' + str(min(new_longitude)),
    #              xy=(new_longitude.index(min(new_longitude)), min(new_longitude)), xytext=(80, -71.07365),
    #              arrowprops=dict(facecolor='red', shrink=0.05),
    #              )
    # plt.annotate('max latitude: ' + str(max(new_latitude)),
    #              xy=(new_latitude.index(max(new_latitude)), max(new_latitude)), xytext=(200, 42.427167),
    #              arrowprops=dict(facecolor='red', shrink=0.05),
    #              )
    # plt.annotate('min latitude: ' + str(min(new_latitude)),
    #              xy=(new_latitude.index(min(new_latitude)), min(new_latitude)), xytext=(300, 42.427158),
    #              arrowprops=dict(facecolor='red', shrink=0.05),
    #              )
    plt.title("Walking UTM Northing")
    # plt.xlabel("message counts")
    # plt.yticks(np.arange(min(new_altitude), max(new_altitude)))
    # plt.ylim(min(new_longitude), max(new_longitude))
    # plt.ylabel("meters(m)")
    plt.show()
