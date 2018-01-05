#!/usr/bin/sh

LOGIN=$USER
sudo echo $LOGIN

# main script
sudo sed s/user/$LOGIN/ < rosboot > /usr/local/bin/rosboot
sudo chmod a+x /usr/local/bin/rosboot

# service
sudo cp rosboot.sh /etc/init.d
sudo chmod a+x /etc/init.d/rosboot.sh
sudo update-rc.d rosboot.sh defaults

