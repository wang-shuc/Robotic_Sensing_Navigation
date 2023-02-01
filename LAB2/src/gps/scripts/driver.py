import rospy
import serial
import utm
import string
from math import sin, pi
from std_msgs.msg import Float64
from gps.msg import GPS_msg


def trans_to_Degree(DDmmm):
    mmData = str(DDmmm).split('.')
    ddData = 0
    i = len(mmData[0])

    for i in range(i, i-2, -1):
        j = int(- (i - len(mmData[0])))
        if j:
            ddData = ddData + float(mmData[0][int(i)-1]) * 10
        else:
            ddData = ddData + float(mmData[0][int(i)-1])

    ddData += float(mmData[1])/pow(10,len(mmData[1]))
    ddData = ddData / 60

    if len(mmData[0]) == 5:
        ddData += float(str(DDmmm)[:3])
    elif len(mmData[0]) == 4:
        ddData += float(str(DDmmm)[:2])
    return ddData




if __name__ == '__main__':
    SENSOR_NAME = "GPS_1"
    rospy.init_node('gpsgps_reader')
    serial_port = rospy.get_param('~port','/dev/ttyACM0')
    serial_baud = rospy.get_param('~baudrate', 57200) 
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    print("Using GPS sensor on port "+serial_port+" at "+str(serial_baud))
    gps_pub = rospy.Publisher(SENSOR_NAME+'/gpsgps', GPS_msg, queue_size=5)
    
    rospy.sleep(0.2)  
          
    line = port.readline()
    
    GPS_msg = GPS_msg()
    GPS_msg.header.frame_id = "gpsgps_GNGGA"
    GPS_msg.header.seq=0
    

    print("Initialization complete!")
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            line=line.decode("utf-8")
            
            #if line.startswith('$GPGGA'):
            if "$GNGGA" in line:
                #print("Line: ")
                #print(line)
                data=line.split(',')
                print(data) 
                if data[2] == '':
                    print("No Data")
                else:
                    utc_=float(data[1])
                    latitude_= trans_to_Degree(float(data[2]))
                    latitude_dir_= data[3]
                    print("latitude_dir:",latitude_dir_)
                    longitude_ = trans_to_Degree(float(data[4]))
                    longitude_dir_ = data[5]
                    print("longitude_dir:",longitude_dir_)
                    GPS_quality = data[6]
                    print("GPS_quality:",GPS_quality)
                    sats = data[7]
                    print("Number of satellites:",sats)
                    hdop = data[8]
                    print("Horizontal dilution of precision:",hdop)
                    altitude_=float(data[9])
                    print("Altitude:",altitude_)
                    if latitude_dir_ =="S":
                    	latitude_=-latitude_
                    if longitude_dir_ == "W":
                    	longitude_=-longitude_
                    
                    utm_ = utm.from_latlon(latitude_, longitude_) 
                    utm_easting_=float(utm_[0])
                    utm_northing_=float(utm_[1])
                    zone_=float(utm_[2])
                    letter_=utm_[3]
                    
                    GPS_msg.latitude = latitude_
                    GPS_msg.longitude = longitude_                    
                    GPS_msg.altitude = altitude_
                    GPS_msg.utm_easting = utm_easting_
                    GPS_msg.utm_northing = utm_northing_
                    GPS_msg.zone = zone_
                    GPS_msg.letter = letter_
                    
                    
            gps_pub.publish(GPS_msg)
                
            
    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
    	rospy.loginfo("Shutting down gps node...")
