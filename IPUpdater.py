
from datetime import datetime
from simplegist import Simplegist
from urllib2 import urlopen
import os



f = os.popen('ifconfig wlan0 | grep "inet\ addr" | cut -d: -f2 | cut -d" " -f1')
addr = str(f.read().strip())
print(addr) #was working with this here? 

gg = Simplegist(username='Ruenzic', api_token='4d794eba5326a8e3b0d552f7a78ed91742cc7325')
current_time = str(datetime.now())
ip = urlopen('http://ipinfo.io/ip').read().strip()
content = current_time + "   " + addr
gg.profile().edit(id='1a9229e4de7ebaa8cc617163a52ced2f', content=content)

#print(content)