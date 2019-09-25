# solowand_unilever

you must install streameye

########

cd ~
sudo git clone https://github.com/ccrisan/streameye
cd ~/streameye
sudo make
sudo make install
cd ~

########

# to get the appropriate demo programs to run,
# you can edit the Makefile to set the -D to __DEMO__, __TEST__, or nothing for
# production which isn't done at this time

sudo cp solo*.service /lib/systemd/system
cd /lib/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable soloeye.service
sudo systemctl daemon-reload

#and either solowand.service for the real wand or
sudo systemctl enable solowand.service
# or this for the simulated wand
sudo systemctl enable solosim.service

# and then reboot to test.  it should be 10 seconds after boot before streameye
# shows up in the ps -A list.  wand should be there as soon as you can run ps -A


