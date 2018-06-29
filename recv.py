import socket

host=''
port=64321
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
s.bind((host, port))
try:
    while True:
        data, addr = s.recvfrom(1024)
        print "Get info From %s:%d:" % addr
        print "%s\n--" % data
except:
    pass
s.close()
