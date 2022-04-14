C:

cd C:\Program Files\FlightGear 2018.3.6

SET FG_ROOT=C:\Program Files\FlightGear 2018.3.6\data

.\\bin\fgfs.exe --fdm=null --native-fdm=socket,in,30,localhost,5502,udp  --disable-terrasync --prop:/sim/rendering/shaders/quality-level=0 --aircraft=747-400 --fog-fastest --disable-clouds --start-date-lat=2021:01:01:12:00:00 --disable-sound --in-air --airport=KSFO --runway=10L --altitude=7224 --heading=113 --offset-distance=4.72 --offset-azimuth=0
