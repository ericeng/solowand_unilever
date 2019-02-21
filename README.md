# solowand_unilever

to get the appropriate demo programs to run,

sudo cp solo*.service /lib/systemd/system
cd /lib/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable solowand.service
sudo systemctl daemon-reload
sudo systemctl enable soloeye.service

and then reboot to test.  it should be 10 seconds after boot before streameye shows up in the ps -A list.  wand should be there as soon as you can run ps -A

