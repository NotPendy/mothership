## Commands for running SITL
-------------------------------------
### For Drone 1
`
sudo docker run -it --rm --env SYSID=0 fixed
`

`
mavproxy.py --master=tcp:172.17.0.2:5760 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14560
`

### For Drone 2
`
sudo docker run -it --rm -p 5760:5760/tcp --env INSTANCE=0 --env SYSID=1 fixed
`

`
mavproxy.py --master=tcp:172.17.0.3:5760 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14561
`

in directory where python file is located :

`
python3 babyship.py --connect 127.0.0.1:14561
`

in directory where python file is located (first connect string is mothership address, second connection string is babyship address)

`
python3 mothership.py --connect 127.0.0.1:14560 --connect2 127.0.0.1:14561
`

-------------------------------------
## Commands for actual running
-------------------------------------
for windows baby gcs

`
mavproxy --master=COM3 --baudrate=460800 --out=localhost:14552 --out=localhost:14561 --out=udpout:192.168.1.140:14562
`

`
python3 babyship.py --connect 192.168.1.156:14561
`

for baby GCS 14552, for baby app 14561, and for mother companion computer 14562

for linux on pi

`
mavproxy.py --master=/dev/ttyACM0  --out=udpout:192.168.1.156:14551 --out=localhost:14560
`

`
mothership.py -connect localhost:14560 --connect2 192.168.1.140:14562   
`

for mother GCS 14551 and for mother app 14560