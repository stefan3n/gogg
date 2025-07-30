import serial.tools
import serial.tools.list_ports

# Run this file to see the name, pid, vid of the connected devices through serial (arduinos, lidar etc.)
def main():

    for i in serial.tools.list_ports.comports():
        print(i.name)
        print(i.pid)
        print(i.vid)
        print("\n")

if __name__=="__main__":
    main()