
from datetime import datetime
from simplegist import Simplegist
from urllib2 import urlopen
import os



f = os.popen('ifconfig wlan0 | grep "inet\ addr" | cut -d: -f2 | cut -d" " -f1')
addr = str(f.read().strip())
#print(addr)

gg = Simplegist(username='Ruenzic', api_token='261b880b2f45a39063aa10e5f72cee604415b8a0')
current_time = str(datetime.now())
ip = urlopen('http://ipinfo.io/ip').read().strip()
content = current_time + "   " + addr
gg.profile().edit(id='1a9229e4de7ebaa8cc617163a52ced2f', content=content)

#print(content)