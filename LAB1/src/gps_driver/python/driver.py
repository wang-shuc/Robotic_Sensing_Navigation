import rospy
import serial
import utm
import string
from math import sin, pi
from std_msgs.msg import Float64
from gps_driver.msg import gps_msg


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
    SENSOR_NAME = "gps"
    rospy.init_node('gps')
    serial_port = rospy.get_param('~port','/dev/pts/2')
    serial_baud = rospy.get_param('~baudrate',4800) 
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    print("Using GPS sensor on port "+serial_port+" at "+str(serial_baud))
    gps_pub = rospy.Publisher('gps', gps_msg, queue_size=5)
    
    rospy.sleep(0.2)  
          
    line = port.readline()
    
    gps_msg = gps_msg()
    gps_msg.Header.frame_id = "GPS1_Frame"
    gps_msg.Header.seq=0
    print("header type:", type(gps_msg.Header.frame_id))

    print("Initialization complete!")
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            line=line.decode("utf-8")
            
            #if line.startswith('$GPGGA'):
            if "$GPGGA" in line:
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
                    
                    gps_msg.Latitude = latitude_
                    gps_msg.Longitude = longitude_                    
                    gps_msg.Altitude = altitude_
                    gps_msg.UTM_easting = utm_easting_
                    gps_msg.UTM_northing = utm_northing_
                    gps_msg.Zone = zone_
                    gps_msg.Letter = letter_
                    
                    
            gps_pub.publish(gps_msg)
                
            
    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
    	rospy.loginfo("Shutting down gps node...")
