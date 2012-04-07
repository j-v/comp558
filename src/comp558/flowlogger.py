class Flow2dLogger:
   def __init__(self, filename, mode='r'):
      self.mode = mode
      self.logfile = open(filename, mode)
      self.next_timestamp = None

   def write(self, timestamp, points, vels):
      if self.mode != 'w': raise Exception('File not open for writing')

      self.logfile.write(str(timestamp)+'\n')
      for (pt,vel) in zip(points, vels):
	 (x,y)=pt
	 (vx,vy)=vel
	 self.logfile.write('{0},{1},{2},{3}\n'.format(x,y,vx,vy))

   def read(self):
      ''' return (timestamp, points, vels) tuple, or None if logfile is completed '''
      if self.next_timestamp:
	 timestamp = self.next_timestamp
      else:
	 l = self.logfile.readline()
	 if not l: return ''
	 else: timestamp = long(l)

      points = []
      vels = []
      l = self.logfile.readline()
      while l and l.find(',') > -1:
	 try:
	    (x,y,vx,vy) = l.split(',')
	    points.append( (int(x),int(y)) )
	    vels.append( (float(vx), float(vy)) )
	 except:
	    raise Exception('Improperly formatted line')
	 l = self.logfile.readline()


      if l:
      	self.next_timestamp = long(l)
      else: self.next_timestamp = None

      return (timestamp, points, vels)

      pass

   def close(self):
      if not self.logfile.closed:
	 self.logfile.close()
      
   def __del__(self):
      if not self.logfile.closed:
        self.logfile.close()



class Vel3dLogger:
   def __init__(self, filename, mode='r'):
      self.mode = mode
      self.logfile = open(filename, mode)
      self.next_timestamp = None

   def write(self, timestamp, points, vels):
      if self.mode != 'w': raise Exception('File not open for writing')

      self.logfile.write(str(timestamp)+'\n')
      for (pt,vel) in zip(points, vels):
	 (x,y,z)=pt
	 (vx,vy,vz)=vel
	 self.logfile.write('{0},{1},{2},{3},{4},{5}\n'.format(x,y,z,vx,vy,vz))

   def read(self):
      ''' return (timestamp, points, vels) tuple, or None if logfile is completed '''
      if self.next_timestamp:
	 timestamp = self.next_timestamp
      else:
	 l = self.logfile.readline()
	 if not l: return ''
	 else: timestamp = long(l)

      points = []
      vels = []
      l = self.logfile.readline()
      while l and l.find(',') > -1:
	 try:
	    (x,y,z,vx,vy,vz) = l.split(',')
	    points.append( (float(x),float(y),float(z)) )
	    vels.append( (float(vx), float(vy),float(vz)) )
	 except:
	    raise Exception('Improperly formatted line')
	 l = self.logfile.readline()


      if l:
      	self.next_timestamp = long(l)
      else: self.next_timestamp = None

      return (timestamp, points, vels)

      pass

   def close(self):
      if not self.logfile.closed:
	 self.logfile.close()
      
   def __del__(self):
      if not self.logfile.closed:
        self.logfile.close()
