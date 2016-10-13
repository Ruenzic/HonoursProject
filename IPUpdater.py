
from datetime import datetime
from simplegist import Simplegist
from urllib2 import urlopen
import os



f = os.popen('ifconfig wlan0 | grep "inet\ addr" | cut -d: -f2 | cut -d" " -f1')
addr = str(f.read().strip())
#print(addr)

gg = Simplegist(username='Ruenzic', api_token='fc29144837eaab82e9835dc9a31ad7e75252580c')
current_time = str(datetime.now())
ip = urlopen('http://ipinfo.io/ip').read().strip()
content = current_time + "   " + addr
gg.profile().edit(id='1a9229e4de7ebaa8cc617163a52ced2f', content=content)

#print(content)