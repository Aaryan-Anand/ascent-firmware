import serial
import csv
from time import sleep, time
import sys

# python3 send_flight_data_to_serial.py | gnuplot -p -e "plot '-' with linespoints notitle"

skip = True

T = 0
B = 2
A = 1

rows = []

with open("booster.csv", "r") as f:
    reader = csv.reader(f)

    for row in reader:
        # skip the first row
        if skip:
            skip = False
            continue
        data = []
        for item in row:
            data.append(float(item))
        rows.append(data)

st = time()
def now(): return time()-st

i = 0

def advance_i():
    global i
    while rows[i][T] < now(): i += 1
    i -= 1

ser = serial.Serial(sys.argv[1], timeout=0.01, baudrate=9600)
print("sending data to", ser.name)

while i < len(rows)-1:
    a = rows[i]
    b = rows[i+1]

    alt = a[B] + (b[B] - a[B])/(b[T] - a[T]) * (now()-a[T])
    # accl = (a[A] + (b[A] - a[A])/(b[T] - a[T]) * (now()-a[T]))*1000
    accl = a[A]*9.81
    print(accl)

    while ser.in_waiting > 0:
        print(ser.read(ser.in_waiting).decode("utf-8"))  # Clear input buffer

    s = f"{alt}, {accl}\n"
    ser.write(s.encode('utf-8'))
    # print("sent", s)

    sleep(0.01)

    advance_i()

ser.close()
