import os
import math
import pandas
import serial
import time

MAX_MEAS = 200
AVG_MEAS = 25
SER_PORT = '/dev/cu.usbmodem146120501'
SER_BAUD = 115200
FILENAME = os.path.join(os.getcwd(), 'acceldata.txt')


class SerialPort:
    """Create and read data from a serial port.

    Attributes:
        read(**kwargs): Read and decode data string from serial port.
    """

    def __init__(self, port, baud=9600):
        """Create and read serial data.

        Args:
            port (str): Serial port name. Example: 'COM4'
            baud (int): Serial baud rate, default 9600.
        """
        if not isinstance(port, str):
            raise TypeError('port must be a string.')

        if not isinstance(baud, int):
            raise TypeError('Baud rate must be an integer.')

        self.port = port
        self.baud = baud

        self.ser = serial.Serial(self.port, self.baud, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
        self.ser.flushInput()
        self.ser.flushOutput()

    def Read(self, clean_end=True) -> str:
        """
        Read and decode data string from serial port.

        Args:
            clean_end (bool): Strip '\\r' and '\\n' characters from string. Common if used Serial.println() Arduino function. Default true

        Returns:
            (str): utf-8 decoded message.
        """
        self.ser.flushInput()
        bytesToRead = self.ser.readline()
        decodedMsg = bytesToRead.decode('utf-8')

        if clean_end:
            decodedMsg = decodedMsg.rstrip()

        return decodedMsg

    def Close(self) -> None:
        """Close serial connection."""
        self.ser.close()


def RecordDataPt(ser: SerialPort) -> tuple:
    """Record data from serial port and return averaged result."""
    ax = ay = az = 0.0
    valid_readings = 0

    for _ in range(AVG_MEAS):
        try:
            raw_data = ser.Read()

            data = raw_data.split(',')

            if len(data) != 3:
                continue

            # Extract the values after the "AccelX:", "AccelY:", "AccelZ:" labels
            ax_now = float(data[0].split(':')[1])
            ay_now = float(data[1].split(':')[1])
            az_now = float(data[2].split(':')[1])

            ax += ax_now
            ay += ay_now
            az += az_now
            valid_readings += 1

        except (ValueError, IndexError) as ve:
            print(f"[ERROR]: Data format issue. Skipping this entry: {data}. Error: {ve}")
            continue
        except Exception as e:
            ser.Close()
            raise SystemExit(f"[ERROR]: Error reading serial connection: {e}")

    if valid_readings == 0:
        raise ValueError("[ERROR]: No valid readings received.")

    return (ax / valid_readings, ay / valid_readings, az / valid_readings)


def List2DelimFile(mylist: list, filename: str, delimiter: str = ',', f_mode='a') -> None:
    """Convert list to Pandas dataframe, then save as a text file."""
    df = pandas.DataFrame(mylist)
    df.to_csv(
        filename,
        sep=delimiter,
        mode=f_mode,
        header=False,  # no col. labels
        index=False  # no row numbers
    )


def main():
    ser = SerialPort(SER_PORT, baud=SER_BAUD)
    data = []

    print('[INFO]: Place sensor level and stationary on desk.')

    time.sleep(0.05)
    for _ in range(MAX_MEAS):
        try:
            ax, ay, az = RecordDataPt(ser)
            magn = math.sqrt(ax**2 + ay**2 + az**2)
            print('[INFO]: Avgd Readings: {:.4f}, {:.4f}, {:.4f} Magnitude: {:.4f}'.format(
                ax, ay, az, magn))
            data.append([ax, ay, az])
        except Exception as e:
            print(f"[ERROR]: Measurement failed: {e}")
            break
        print('[INFO]: Saving data and exiting...')
        List2DelimFile(data, FILENAME, delimiter='\t')
        ser.Close()
        print('[INFO]: Done!')
        return

    print('[WARNING]: Reached max. number of datapoints, saving file...')
    List2DelimFile(data, FILENAME, delimiter='\t')
    ser.Close()
    print('[INFO]: Done!')


if __name__ == '__main__':
    main()
