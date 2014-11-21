from xmlrpclib import ServerProxy
import os, sys
total=len(sys.argv) 
if total < 2:
	print 'rosrv -list\nrosrv -rvstate\nrosrv -enable [monitors]\nrosrv -disable [monitors]'
else:
	master = ServerProxy(os.environ['ROS_MASTER_URI'])
	#print master.getSystemState('/')
        mode = sys.argv[1];
        if(mode=='-rvstate'):
		print master.getRVState('rosrv')
        elif(mode=='-list'):
                print master.getMonitors('rosrv')[2]
        else:
		monitors = sys.argv[2:]
        	if(mode=='-enable'):
			print master.monitorControl('rosrv','enable',monitors)
        	elif(mode=='-disable'):
			print master.monitorControl('rosrv','disable',monitors);
