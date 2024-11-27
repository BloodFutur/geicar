# This file describes the steps to configure the Raspberry PI, from the OS to the ROS installation

## 1. Raspberry configuration from the SD card image
**An image of the SD card with all the necessary configurations and the initial ROS nodes is available here : .....**

1. Flash the image on your SD Card (you can use etcher)
2. Insert the SD card in the Raspberry and Power ON the board (login = pi, password = geicar)

3. Two changes are necessary to complete the configuration:
* In the file /etc/dhcp/dhcpd.conf, replace the MAC address assigned to the address 192.168.1.10 by the MAC address of the Jetson board that will be used (interface eth0)
* In the file ~/.bashrc, change the line "export ROS_DOMAIN_ID = 1" by "export ROS_DOMAIN_ID = XX", with XX is the car number (exemple : XX=1 for the car n°1)

4. Reboot the board

**The RaspberryPi is ready**

## 2.Raspberry configuration from an empty SD card
### OS Installation
1. Download Ubuntu 22.04 Server for arm64 : https://ubuntu.com/download/server/arm
2. Flash the image on your SD Card (you can use etcher)
3. Insert the SD card in the Raspberry and Power ON the board

### Keyboard configuration
Change the keyboard layout in /etc/default/keyboard. Then reboot.

### Network configuration - Wifi
This procedure is extracted from "https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#4-boot-ubuntu-server"

1. Copy the file "/geicar/RaspberryPI3/configuration_files/50-cloud-init.yaml" to "/etc/netplan/50-cloud-init.yaml". **Adapt SSID and password if necessary**

2. Apply changes :
```sh
sudo netplan apply
sudo reboot
```
3. After reboot, you can see the list of available wifi networks : 
```sh
sudo wpa_cli -i wlan0 list_networks
```
To select a wifi network : 
```sh
sudo wpa_cli -i wlan0 select_network <network_id>
```

### Updates and tools installations
1. General update :
```sh
sudo apt update
sudo apt upgrade
```

2. SSH tools:
```sh
sudo apt install openssh-server
sudo service ssh enable
sudo service ssh start
```

3. Network configuration tools :
```sh
sudo apt install net-tools
```

4. CAN tools :
```sh
sudo apt install can-utils
```

5. Nano :
```sh
sudo apt install nano
```

### Firewall configuration
1. Allow all ssh connections :
```sh
sudo ufw allow ssh
```

2. Allow multicast :
```sh
route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
ifconfig eth0 multicast
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```

3. Allow communication with Jetson Nano :
```sh
sudo ufw allow from 192.168.1.10
```

### CAN configuration - PiCan2

1. Copy the file "/geicar/RaspberryPI3/configuration_files/config.txt" to "/boot/firmware/config.txt"

2. Copy the file "/geicar/RaspberryPI3/configuration_files/pican2.service" to "/lib/systemd/system/pican2.service"

3. Enable the pican2 service :
```sh
sudo systemctl enable pican2
sudo reboot
```
After reboot, check that the service has started correctly :
```sh
sudo systemctl status pican2
```

### Installation of dhcp server (for ethernet)
This procedure is extracted from "https://www.linuxfordevices.com/tutorials/ubuntu/dhcp-server-on-ubuntu"

1. Install isc-dhcp-server :
```sh
sudo apt install isc-dhcp-server
```
2. Make a backup of the "dhcpd.conf" file :
```sh
sudo mv /etc/dhcp/dhcpd.conf{,.backup}
```
3. Copy the file "/geicar/RaspberryPI3/configuration_files/dhcpd.conf" to "/etc/dhcp/dhcpd.conf" and adapt the MAC address (line 12) according to the Jetson Nano Eth0 interface 

4. Copy the file "/geicar/RaspberryPI3/configuration_files/isc-dhcp-server" to "/etc/default/isc-dhcp-server"

5. Copy the file "/geicar/RaspberryPI3/configuration_files/99_config.yaml" to "/etc/netplan/99_config.yaml" to add a fixed @IP address to the eth0 interface
 
6. Apply changes :
```sh
sudo netplan apply
sudo reboot
```

7. After reboot, you can ping the Jetson board to check the configuration :
```sh
ping 192.168.1.10
```

### Share Internet connection from wlan0 to eth0 (to give internet access to the Jetson)

1. Enable ip forwarding :
```sh
sudo sysctl -w net.ipv4.ip_forward=1
```

2. Configure iptables :
```sh
sudo iptables -A FORWARD --in-interface wlan0 -j ACCEPT
sudo iptables --table nat -A POSTROUTING --out-interface eth0 -j MASQUERADE
 ```
 
3. In the file "/etc/ufw/sysctl.conf", comment out the line net.ipv4.ip_forward=1.

4. In the file "/etc/default/ufw", change the corresponding line to : DEFAULT_FORWARD_POLICY="ACCEPT"

5. Copy the file /geicar/RaspberryPI3/configuration_files/before.rules" to "/etc/ufw/before.rules"

6. Run :
```sh
sudo systemctl restart
sudo ufw enable
sudo reboot
```

### Change Login and Hostname (by "pi" and "geicar")
This procedure is extracted from "https://www.hepeng.me/changing-username-and-hostname-on-ubuntu/"

**LOGIN : pi**
1. Login using your username and password
2. Set "geicar" has password for the "root" account :
```sh
sudo passwd root 

3. Log out :
```sh
exit
```

4. Log in using the "root" account and the password you set earlier
5. Change the username and home folder to the new name you want (pi):
```sh
usermod -l <newname> -d /home/<newname> -m <oldname>
```
6. Change the group name to the new name you want (pi):
```sh
groupmod -n <newgroup> <oldgroup> 
```
7. Lock down the "root" account :
```sh
passwd -l root
```
8. Logout :
```sh
exit
```

**HOSTNAME : geicar**

1. In "/etc/hostname", delete the old name and configure the new one (geicar)
2. In "/etc/hosts", replace all occurrences of the existing computer name with your new name (geicar)
3. Reboot the system for the changes to take effect:
```sh
sudo reboot
```

### Add SWAP Space
It is useful to add a swap, because the raspberry has only 1GB of RAM, which is not much for compilations. A swap of 2048MB seems to be enough. The procedure to add a swap can be found here : https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-22-04


### Installation of ROS2 Humble
1. Follow the installation procedure of ROS2 humble available here : "https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"

2. Install rqt package :
```sh
sudo apt update
sudo apt install ~nros-humble-rqt*
```
3. Install rosdep tools :
```sh
sudo apt install python3-rosdep2
update rosdep
```
4. Install the Raspberry ROS2 workspace : copy the [ros2_ws](./ros2_ws) to ~/ros2_ws 

5. Initialize rosdep dependencies **(in ~/ros2_ws)** :
```sh
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

6. Install colcon tools :
```sh
sudo apt install python3-colcon-common-extensions
```

7. Compile and source :
```sh
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/local_setup.bash
source /opt/ros/humble/setup.bash
```

8. Source the ROS environment and set the ROS_DOMAIN_ID\
**Replace XX by the car number (1,2,3,4 ...). Example : "export ROS_DOMAIN_ID=1" for the car n°1**
```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=XX" >> ~/.bashrc
```

### Automatic time/date synchronization

1. Switch to root user `su`
2. Copy the `time_update.sh` file to your root directory (`/root/time_update.sh`)
3. Add the executable right to the file: `chmod +x time_update.sh`
4. Add a cronjob with: `crontab -e` (be sure to be connected as root or do `sudo` otherwise)
5. In the file opend by the previous command, add the following: `@reboot sh $HOME/time_update.sh`