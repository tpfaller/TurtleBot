from datetime import datetime
import time

while True:
    now = datetime.now()
    dt_string = now.strftime("%Y/%m/%d %H:%M:%S")
    print(dt_string)
    time.sleep(1)
