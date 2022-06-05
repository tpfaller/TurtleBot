#!/usr/bin/env python3
from multiprocessing.connection import Listener, Connection
from multiprocessing.reduction import ForkingPickler
import threading

def handle_client(conn: Connection):
    try:
        while True:
            # image = conn.recv()
            # Workaround for python2 comppatibility
            conn._check_closed()
            conn._check_readable()
            buf = conn._recv_bytes()
            image = ForkingPickler.loads(buf.getbuffer(), encoding='bytes')
            
            # TODO: Get object positions from image

            objects = {
                'obstacle_0': (100, 200),
                'hulk': (300, 150)
            }

            # conn.send(objects)
            # Workaround for python2 comppatibility
            conn._check_closed()
            conn._check_writable()
            conn._send_bytes(ForkingPickler.dumps(objects, protocol=2))
    finally:
        conn.close()

if __name__ == '__main__':
    address = ('localhost', 6000)
    listener = Listener(address)
    try:
        while True:
            conn = listener.accept()
            client_thead = threading.Thread(target=handle_client, args=(conn,), daemon=True)
            client_thead.start()
    finally:
        listener.close()
