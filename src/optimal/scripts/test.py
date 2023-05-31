import os
import time

if __name__ == '__main__':
    try:
        while True:
            time.sleep(0.1)
    except exit(137):
        with open('txt.txt','a') as f:
            f.write('111')

