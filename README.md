# hrm_ros

Heart rate monitoring for ROS. So far tested on Ubuntu 14.04 (Indigo) and 18.04 (Melodic) and with Polar H10 (node.launch) and Xiaomi Mi Band 2 (miband2.py).

Following commands are needed to allow normal (non-root) user to run it:

```
sudo apt-get install libcap2-bin
sudo setcap 'cap_net_raw,cap_net_admin+eip' `which hcitool`
```

Dependencies:

```
sudo pip install bluepy
```
