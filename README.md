# Pi3DLidar

## 🛠️ Project Overview
This repository contains code, configurations, 3D Models, and documentation for creating a 3D room mapping system using **ROS 2 Humble Hawksbill**, **Raspberry Pi 3B+**, and a **YDLIDAR X2**. The system is based on **Ubuntu 22.04 Server**, since the 3B+ does not have the RAM to run any Ubuntu Desktop applications and will primarily be interacted with through an X11 session.

**TODO: Pretty picture gif here**

## 📖 Table of Contents
- [Project Overview](#-project-overview)
  - [Hardware Components](#-hardware-components)
  - [Software Requirements](#-software-requirements)
- [How It Works](#-how-does-it-work?)
- [Installation Instructions](#-setup)
- [Usage Guide](#-usage-guide)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [License](#-license)



## 🔌 Hardware Components
This project runs on the following hardware:

| Component        | Description                     |
|-------------------|---------------------------------|
| Raspberry Pi 3B+ | 1.4GHz 64-bit quad-core ARM CPU |
| MicroSD Card     | 32GB or larger recommended      |
| Micro-USB Adapters     | 2x PWR, 1x Data      |
| [YDLidar X2](https://www.amazon.com/SmartFly-info-YDLIDAR-Scanner-Ranging/dp/B07W613C1K?dib=eyJ2IjoiMSJ9.8PG4-1hWigQu-gsvl8iE5Yif00ngk04n3Kx6fCfFHOCmxTAMrObxOLjcjGw7G7HifXIikd7D167cP6B2P4rhHxMCMrxS1A6YWC4wVxMNii-qXfTaS4He9nga8KHTqH23LpdI8J26UPH6n54EDMCEh81pi2U39Cid0A038iVFCTNJlk-nZyZOVN6x5vKO5q3zoPnwSr_MiNDek5POyoj0NzbPx2pxq_Aw-5dty1qFRjYBmo7JClxzuLLzUA4oz3Ol5nk0k7Msv8LnrTQdyChFQw.Mn1ZRxb2oOb1J4asP6YPO_T4NM9FtmchY9i6onNb3L4&dib_tag=se&keywords=ydlidar&qid=1739663408&s=electronics&sr=1-8) | Primary Component for this project      |
| [5V Stepper Motor](https://www.amazon.com/dp/B015RQ97W8?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_1) | Any stepper motor and driver will do, this one had the words 'Arduino' and 'RPi' attached to it. |
| (Optional) 8000mAH USB Battery Pack | You can use a few USB wall plugs just as well, but I liked the idea of making it portable. |

**Additional Notes:** Add any custom sensor or actuator details here.

## 🖥️ Software Requirements
- **Operating System:** Ubuntu 22.04 Server (64-bit)
- **ROS 2 Distribution:** Humble Hawksbill
- **Python:** 3.10+
- **Additional Packages:**
  - `ros-humble-desktop`
  - `x11-apps`, `xauth`, `openbox`

# How Does it Work?

## Why Bother?
Because I have too many random robotics components collecting dust in my garage, and LIDAR is cool :).

# Setup

## ⚙️ Raspberry Pi and Software Setup

### 1️⃣ Raspberry Pi Setup
Since a Raspberry Pi 3B+ is what I had laying around, that's what I used for this project. I don't think I can reccomend anything older, but more powerful Pis are always mo' better. I highly recommend using the [RPi Imager Application](https://www.raspberrypi.com/software/) when imaging your SD card. 

Using this application, install Ubuntu 22.04 onto your microSD card- I also recomend adding your WiFi credentials and enabling SSH as part of the initial setup if you haven't already. We will be using the Raspberry Pi in a headless mode for this project.

### 2️⃣ Getting Into The Pi
Once you've successfully imaged Ubuntu 22.04 onto the microSD and powered on the Pi, you may need to inititally plug in a monitor and keyboard to the Pi, log in to a tty (Ctrl+Alt+F2), and run `ip addr` to get the address the Pi is on your network. You can continue the setup via directly, but it will become important later on to know this address to access the ROS GUI. 

Once you have the IP address, you can SSH into the Pi with a computer on the same network *(that allows SSH)* using:
```
ssh <username>:<IP Address>
```

### 3️⃣ Clone the Repository and Setup Dependencies
Once you've logged into the Pi in some manner and connected to the internet, clone this repository onto the Pi:

```bash
git clone https://github.com/joshjab/pi3dlidar.git
cd pi3dlidar
```

For convenience, there is a bash script to install the necessary dependencies. On my Pi 3B+, this takes several hours. There's a small hack in there so you don't have to baby the install and type in your password every so often- probably not the most secure but it works.

```bash
util/install_ros.sh
```

### Verify ROS and X11 Operation
Now that all the dependencies are set up, we want to just verify basic operation of ROS and our X11 setup.

If you run into errors here, see the [Troubleshooting](#-troubleshooting) section.

### Install Pi3DLidar Service

Your Pi should now be ready to go! Now we can finally set up the Pi3DLidar as a `systemd` service so it will start up whenever the Pi is powered on.

## Enclosure
**TODO**: Guide and resources for printing and mounting the enclosure and stand.

## Hardware Setup
**TODO**: Stuff about plugging in LIDAR and Stepper motor. Maybe some crazy I2C stuff for an accelerometer.

# 🚀 Basic Usage Guide

**TODO**: 

## 🚀 Usage Guide (Detailed Wiki)

**TODO**: 

# ❗ Troubleshooting

**1. SSH X11 Forwarding Not Working**
- Ensure the X server is running on the client machine.
- Verify the `DISPLAY` environment variable is set:
  ```bash
  echo $DISPLAY
  ```

**2. ROS 2 Communication Issues**
- Check network settings and multicast permissions.
- Use `ros2 doctor --report` to diagnose common issues.

# 🤝 Contributing
I don't plan to maintain this repository, but feel free to fork and edit as you see fit! If you post something in the [Issues]() tab I might check it one day.

# ⚖️ License
This project is licensed under the MIT License. See the `LICENSE` file for details.


