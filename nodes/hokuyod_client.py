#!/usr/bin/python

#
# python interface to hokuyod
#
from socket import *
import struct
import time
from threading import Thread

class HokuyoMuxError(Exception):
   def __init__(self, value):
      self.value = value
   def __str__(self):
      return repr(self.value) 
 
class HokuyoMux(Thread):
   hok_fmt = "<LLLHHHHHHL"
   data_fmt = "<HH"
   offset = struct.calcsize(hok_fmt)
   datalen = struct.calcsize(data_fmt)
   tick = None

   def __init__(self, host, port):
      Thread.__init__(self)
      self.host = host
      self.port = port
      self.sock = None
      self.ts = None
      self.start()
      #self.enable_hokuyo(False)

   def run(self):
      print "hokuyo: background thread started"
      while True:
        if self.sock and self.tick:
           #print "checking tick:%dms" % (time.time()-self.tick)
           if (time.time()-self.tick) > 5.0:
              print "hokuyo: Closing unused socket"
              self.close()
        time.sleep(1)

   # NB: not called anymore as hokuyod takes on power management of the hokuyo
   def enable_hokuyo(self, bEnable):
      try:
         f = open('/sys/class/gpio/gpio18/value', 'w')
         if (bEnable):
            print "hokuyo: enabling"
            f.write('1\n')
         else:
            print "hokuyo: disabling"
            f.write('0\n')
         f.close
      except:
         pass

   def connect(self):
      print "hokuyo: Opening connection to hokuyod: %s:%d" % (self.host, self.port)
      #self.enable_hokuyo(True)
      try:
         sock = socket(AF_INET, SOCK_STREAM)
         sock.connect((self.host, self.port))
         # put hokuyod into single-shot mode
         sock.send('q\n')
         sock.settimeout(0.5)
         return sock
      except error, msg:
         raise HokuyoMuxError(msg[1])

   def close(self):
      self.sock.close()
      self.sock = None
      #self.enable_hokuyo(False)

   def read(self, latency=100):
      if self.sock == None:
         self.sock = self.connect() 
      self.tick = time.time()
      self.sock.send('q\n')
      while True:
         try:
            data = self.sock.recv(2756)
         except timeout:
            self.sock.close()
            self.sock = None
            raise HokuyoMuxError('timeout')
         except error, msg:
            self.sock.close()
            self.sock = None
            raise HokuyoMuxError(msg)
         try:
            t = struct.unpack(self.hok_fmt, data[0:self.offset])
            datadict = {'magic': '0x%04x' % t[0],
                        'host_time': t[1],
                        'urg_time': t[2],
                        'min_idx': t[3],
                        'max_idx': t[4],
                        'mid_idx': t[5],
                        'tot_idx': t[6],
                        'min_dist': t[7],
                        'max_dist': t[8],
                        'CRC': t[9]}
            if not self.ts:
               self.ts = t[1]
               self.ts_local = time.time()*1000
            localt = self.ts + time.time()*1000 - self.ts_local
            deltat = localt - t[1]
            if deltat > latency:
               continue
            string = ''
            samples = t[4] - t[3] + 1
            datadict["readings"] = []
            for i in range(samples):
               t = struct.unpack_from(self.data_fmt, data, self.offset+(i*self.datalen))
               datadict["readings"].append(t[0])
            datadict["deltat"] = deltat
            return datadict
         except struct.error:
            self.sock.close()
            self.sock = None
            raise HokuyoMuxError('invalid data')
   

if __name__ == "__main__":
   hokuyo = HokuyoMux('localhost', 1338)
   while True:
      data = hokuyo.read(100) # read until data within 100ms
      print data
      time.sleep(.2)
