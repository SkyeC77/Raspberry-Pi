import socket
import time


HOST = '<broadcast>'
PORT = 64321
ADDR = (HOST, PORT)

if __name__=='__main__':
    time.sleep(10)
    ucs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ucs.bind(('', 0))
    ucs.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    while True:
        info = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
        print(info)
        ucs.sendto(info, ADDR)
        time.sleep(2)
    ucs.close()

