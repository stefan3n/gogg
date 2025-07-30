### [< back](../GuideForDocumentation.md)
# Remote connection

### Because you can't have a monitor, keyboard and mouse connected at all times on the car, you will need to connect remotely to the Jetson.
### This is done by connecting through SSH. First, make sure that the Jetson has the right tool installed:
> sudo apt-get install openssh-server

### Now, to connect to the Jetson, make sure you are on the same network and you know the Jetson's local ip address (if you are connected to the Jetson's Hotspot, it's ip will be *10.42.0.1*).

### Now, to connect run this command:
> ssh toycar@10.42.0.1

### With ssh, you have access to a terminal running on the target machine, logged in as that user.

## Linux
### To connect from a Linux machine, make sure you have openssh installed.
> sudo apt-get install openssh
### Don't know if this is necessary, but also try this:
> sudo apt-get install openssh-client

## Windows
### You could try to install a command line tool to use it directly in *cmd* and *PowerShell*.
### What we did is we used *MobaXterm*, which can be downloaded from [here](https://mobaxterm.mobatek.net/). In here you can run the same ssh command as above.

## Static IP
### Incase you want to connect to a router (maybe to forward internet from a phone's hotspot), it would be useful to setup a static IP for the Jetson, otherwise, it might change when restarting the router.
### We tried setting a static IP by messing around with the setting on the Jetson, but it didn't work for us. What we did is we configured directly from our router's interface (accessed through a broser at address *192.168.0.1*).

## Changing network from command line
### To see all known networks use this command:
> sudo nmcli connection show
### To connect to a specific network run:
> sudo nmcli connection up \<network_name\>
### To turn on Hotspot run:
> sudo nmcli connection up Hotspot
### To end a connection run:
> sudo nmcli connection down \<network_name\>


## Using RVIZ remotely
### If you want to visualize data in Rviz while connected trough ssh, append these lines into *.bashrc*:

### On the Jetson running the ROS nodes (might now be required):
> export ROS_IP=10.42.0.1 # local ip address

### On the machine where you are running Rviz:
> export ROS_IP=10.42.0.42 # replace with your local adress
> export ROS_MASTER_URI=http://10.42.0.1:11311 # replace with your Jetson's ip

### Now you can normally run Rviz and it will automatically connect to the ROS nodes on the Jetson.