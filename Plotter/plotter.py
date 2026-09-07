import serial
import matplotlib.pyplot as plt
import csv
import time
from collections import deque
from datetime import datetime


# ============================================================
# SETTINGS
# ============================================================

SERIAL_PORT = "COM7"
BAUD_RATE = 115200

# Number of samples visible in the live graph
MAX_POINTS = 1000

# CSV output file
CSV_FILE = "imu_qvar_data.csv"


# ============================================================
# SERIAL CONNECTION
# ============================================================

ser = serial.Serial(
    SERIAL_PORT,
    BAUD_RATE,
    timeout=1
)

time.sleep(2)

# Remove anything already sitting in the serial buffer
ser.reset_input_buffer()

print(f"Connected to {SERIAL_PORT}")
print(f"Saving data to: {CSV_FILE}")
print("Waiting for sensor data...")


# ============================================================
# CSV FILE
# ============================================================

csv_file = open(
    CSV_FILE,
    "w",
    newline="",
    buffering=1
)

writer = csv.writer(csv_file)

writer.writerow([
    "PC_Time",
    "timestamp",
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz",
    "qvar"
])

# Force header to disk
csv_file.flush()


# ============================================================
# LIVE DATA BUFFERS
# ============================================================

t = deque(maxlen=MAX_POINTS)

acc_x = deque(maxlen=MAX_POINTS)
acc_y = deque(maxlen=MAX_POINTS)
acc_z = deque(maxlen=MAX_POINTS)

gyro_x = deque(maxlen=MAX_POINTS)
gyro_y = deque(maxlen=MAX_POINTS)
gyro_z = deque(maxlen=MAX_POINTS)

qvar = deque(maxlen=MAX_POINTS)


# ============================================================
# CREATE FIGURE
# ============================================================

plt.ion()

fig, (acc_ax, gyro_ax, qvar_ax) = plt.subplots(
    3,
    1,
    figsize=(12, 9)
)


# ============================================================
# ACCELEROMETER
# ============================================================

acc_line_x, = acc_ax.plot([], [], label="X")
acc_line_y, = acc_ax.plot([], [], label="Y")
acc_line_z, = acc_ax.plot([], [], label="Z")

acc_ax.set_title("Accelerometer")
acc_ax.set_ylabel("Acceleration (mg)")
acc_ax.grid(True)
acc_ax.legend(loc="upper right")


# ============================================================
# GYROSCOPE
# ============================================================

gyro_line_x, = gyro_ax.plot([], [], label="X")
gyro_line_y, = gyro_ax.plot([], [], label="Y")
gyro_line_z, = gyro_ax.plot([], [], label="Z")

gyro_ax.set_title("Gyroscope")
gyro_ax.set_ylabel("Angular velocity (dps)")
gyro_ax.grid(True)
gyro_ax.legend(loc="upper right")


# ============================================================
# QVAR
# ============================================================

qvar_line, = qvar_ax.plot(
    [],
    [],
    label="QVAR"
)

qvar_ax.set_title("QVAR")
qvar_ax.set_xlabel("Time (s)")
qvar_ax.set_ylabel("QVAR (mV)")
qvar_ax.grid(True)
qvar_ax.legend(loc="upper right")


plt.tight_layout()


# ============================================================
# VARIABLES
# ============================================================

sample_count = 0
start_timestamp = None


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:

        # ----------------------------------------------------
        # READ ONE LINE FROM ESP32
        # ----------------------------------------------------

        line = ser.readline().decode(
            "utf-8",
            errors="ignore"
        ).strip()

        if not line:
            continue


        # ----------------------------------------------------
        # PARSE CSV
        #
        # timestamp,ax,ay,az,gx,gy,gz,qvar
        # ----------------------------------------------------

        try:

            values = line.split(",")

            if len(values) != 8:
                continue

            timestamp = float(values[0])

            AX = float(values[1])
            AY = float(values[2])
            AZ = float(values[3])

            GX = float(values[4])
            GY = float(values[5])
            GZ = float(values[6])

            QVAR = float(values[7])

        except ValueError:

            # Ignore header/debug messages
            continue


        # ----------------------------------------------------
        # CONVERT TIMESTAMP TO SECONDS
        # ----------------------------------------------------

        if start_timestamp is None:
            start_timestamp = timestamp

        elapsed_time = (
            timestamp - start_timestamp
        ) / 1000.0


        # ----------------------------------------------------
        # STORE IN BUFFERS
        # ----------------------------------------------------

        t.append(elapsed_time)

        acc_x.append(AX)
        acc_y.append(AY)
        acc_z.append(AZ)

        gyro_x.append(GX)
        gyro_y.append(GY)
        gyro_z.append(GZ)

        qvar.append(QVAR)


        # ====================================================
        # SAVE TO CSV
        # ====================================================

        pc_time = datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S.%f"
        )

        writer.writerow([
            pc_time,
            timestamp,
            AX,
            AY,
            AZ,
            GX,
            GY,
            GZ,
            QVAR
        ])

        # Make sure data is continuously written to disk
        csv_file.flush()

        sample_count += 1


        # ====================================================
        # UPDATE ACCELEROMETER
        # ====================================================

        acc_line_x.set_data(t, acc_x)
        acc_line_y.set_data(t, acc_y)
        acc_line_z.set_data(t, acc_z)


        # ====================================================
        # UPDATE GYROSCOPE
        # ====================================================

        gyro_line_x.set_data(t, gyro_x)
        gyro_line_y.set_data(t, gyro_y)
        gyro_line_z.set_data(t, gyro_z)


        # ====================================================
        # UPDATE QVAR
        # ====================================================

        qvar_line.set_data(t, qvar)


        # ====================================================
        # X AXIS
        # ====================================================

        if len(t) > 1:

            xmin = t[0]
            xmax = t[-1]

            if xmax <= xmin:
                xmax = xmin + 1

            acc_ax.set_xlim(xmin, xmax)
            gyro_ax.set_xlim(xmin, xmax)
            qvar_ax.set_xlim(xmin, xmax)


        # ====================================================
        # ACCELEROMETER Y AXIS
        # ====================================================

        if len(acc_x) > 1:

            minimum = min(
                min(acc_x),
                min(acc_y),
                min(acc_z)
            )

            maximum = max(
                max(acc_x),
                max(acc_y),
                max(acc_z)
            )

            margin = max(
                (maximum - minimum) * 0.10,
                10
            )

            acc_ax.set_ylim(
                minimum - margin,
                maximum + margin
            )


        # ====================================================
        # GYROSCOPE Y AXIS
        # ====================================================

        if len(gyro_x) > 1:

            minimum = min(
                min(gyro_x),
                min(gyro_y),
                min(gyro_z)
            )

            maximum = max(
                max(gyro_x),
                max(gyro_y),
                max(gyro_z)
            )

            margin = max(
                (maximum - minimum) * 0.10,
                1
            )

            gyro_ax.set_ylim(
                minimum - margin,
                maximum + margin
            )


        # ====================================================
        # QVAR Y AXIS
        # ====================================================

        if len(qvar) > 1:

            minimum = min(qvar)
            maximum = max(qvar)

            difference = maximum - minimum

            if difference < 1:

                center = (
                    minimum + maximum
                ) / 2

                minimum = center - 5
                maximum = center + 5

            else:

                margin = difference * 0.10

                minimum -= margin
                maximum += margin

            qvar_ax.set_ylim(
                minimum,
                maximum
            )


        # ====================================================
        # REDRAW
        # ====================================================

        fig.canvas.draw()
        fig.canvas.flush_events()


        # ====================================================
        # STATUS
        # ====================================================

        if sample_count % 100 == 0:

            print(
                f"Samples: {sample_count:6d} | "
                f"ACC: ({AX:7.2f}, {AY:7.2f}, {AZ:7.2f}) mg | "
                f"GYR: ({GX:6.2f}, {GY:6.2f}, {GZ:6.2f}) dps | "
                f"QVAR: {QVAR:8.2f} mV"
            )


# ============================================================
# STOP WITH CTRL+C
# ============================================================

except KeyboardInterrupt:

    print("\nStopping recording...")


finally:

    # Make sure everything is saved
    csv_file.flush()
    csv_file.close()

    ser.close()

    plt.ioff()

    print()
    print("======================================")
    print("Recording stopped")
    print(f"Samples recorded: {sample_count}")
    print(f"CSV saved as: {CSV_FILE}")
    print("======================================")

    plt.show()