cp ../pidar/slam_start.sh ~/
chmod +x ~/slam_start.sh
sudo cp ../pidar/PiDAR.service /etc/systemd/system/
sudo systemctl enable PiDAR.service