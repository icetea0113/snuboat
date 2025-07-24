mission_code = 0x00410000  # Example mission code
sensor_mode = (mission_code & 0x00F00000) >> 20
if sensor_mode == 0x1:
    print("Qualisys")
elif sensor_mode == 0x2:
    print("SLAM")
elif sensor_mode == 0x3:
    print("GPS-RTK")
elif sensor_mode == 0x4:
    print("Marker")
print(0x4)