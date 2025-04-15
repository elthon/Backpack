import sys
import time
import csv
import serial
import random
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import json
import datetime
from timezonefinder import TimezoneFinder
import pytz
import os

_crc_tab = [
    0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54,0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
    0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06,0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
    0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0,0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
    0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2,0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
    0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9,0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
    0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B,0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
    0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D,0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
    0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F,0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
    0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB,0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
    0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9,0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
    0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F,0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
    0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D,0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
    0x72,0xA7,0x0D,0xD8,0x8C,0x59,0xF3,0x26,0x5B,0x8E,0x24,0xF1,0xA5,0x70,0xDA,0x0F,
    0x20,0xF5,0x5F,0x8A,0xDE,0x0B,0xA1,0x74,0x09,0xDC,0x76,0xA3,0xF7,0x22,0x88,0x5D,
    0xD6,0x03,0xA9,0x7C,0x28,0xFD,0x57,0x82,0xFF,0x2A,0x80,0x55,0x01,0xD4,0x7E,0xAB,
    0x84,0x51,0xFB,0x2E,0x7A,0xAF,0x05,0xD0,0xAD,0x78,0xD2,0x07,0x53,0x86,0x2C,0xF9,
]

def crsf_frame_crc(frame: bytes) -> int:
    crc_frame = frame[2:-1]
    return crsf_crc(crc_frame)

def crsf_crc(data: bytes) -> int:
    crc = 0
    for i in data:
        crc = _crc_tab[crc ^ i]
    return crc

def idle(duration: float, uart: serial.Serial, received_data: list):
    if uart:
        start = time.monotonic()
        while time.monotonic() - start < duration:
            b = uart.read(128)
            if len(b) > 0:
                received_data.append(b)
                sys.stdout.write(b.decode('ascii', 'replace'))
    else:
        time.sleep(duration)

def output_Crsf(uart: serial.Serial, data):
    if uart is None:
        return
    uart.write(data)

def output_Msp(uart: serial.Serial, data):
    if uart is None:
        return
    buf = b'$X<\x00'
    buf += int.to_bytes(0x0011, length=2, byteorder='little', signed=False)
    buf += int.to_bytes(len(data), length=2, byteorder='little', signed=False)
    buf += data
    buf += int.to_bytes(crsf_crc(buf[3:]), length=1, byteorder='big', signed=False)
    uart.write(buf)

gps_data = []
is_processing = False
tf = TimezoneFinder()

def save_gps_data():
    """Save gps_data to a JSON file."""
    try:
        with open('gps_data.json', 'w') as f:
            json.dump(gps_data, f)
    except Exception as e:
        print(f"Error saving gps_data: {e}")

def load_gps_data():
    """Load gps_data from a JSON file."""
    global gps_data
    try:
        if os.path.exists('gps_data.json'):
            with open('gps_data.json', 'r') as f:
                gps_data = json.load(f)
    except Exception as e:
        print(f"Error loading gps_data: {e}")

def get_local_time(timestamp_us, lat, lon, base_time):
    """Convert timestamp (us) to local time based on coordinates and base time."""
    try:
        time_s = timestamp_us / 1e6
        utc_dt = base_time + datetime.timedelta(seconds=time_s)
        timezone_str = tf.timezone_at(lat=lat, lng=lon)
        if timezone_str:
            timezone = pytz.timezone(timezone_str)
            local_dt = utc_dt.astimezone(timezone)
            return local_dt.strftime('%Y-%m-%d %H:%M:%S')
        return utc_dt.strftime('%Y-%m-%d %H:%M:%S') + " (UTC)"
    except (ValueError, OSError):
        return f"T+{time_s:.3f}s"

def processFile(fname, interval, port, baud, jitter, output, base_time):
    global gps_data, is_processing
    random.seed()
    received_data = []

    if port is not None:
        try:
            uart = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            idle(1.0, uart, received_data)
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            is_processing = False
            return
    else:
        uart = None

    try:
        with open(fname, 'r') as csv_file:
            reader = csv.reader(csv_file)
            next(reader)
            lastTime = None
            for row in reader:
                time_us = float(row[0])
                if lastTime is not None:
                    dur = (time_us - lastTime) / 1e6
                    if dur < interval:
                        continue
                    j = random.randrange(0, int(jitter * 1000.0)) / 1000.0 if jitter > 0 else 0
                    idle(dur + j, uart, received_data)
                sats = int(row[2])
                lat = float(row[3])
                lon = float(row[4])
                alt = int(row[5]) + 1000
                spd = int(float(row[6]) * 360)
                hdg = int(float(row[7]) * 100)
                ground_course = float(row[7])
                time_str = get_local_time(time_us, lat, lon, base_time)
                print(f'{time_str} GPS: ({lat},{lon}) {alt-1000}m {sats}sats Course: {ground_course}°')
                gps_data.append({
                    'lat': lat,
                    'lng': lon,
                    'alt': alt-1000,
                    'sats': sats,
                    'time': time_str,
                    'ground_course': ground_course
                })
                save_gps_data()  # Save after each point
                buf = bytearray(b'\xc8\x00\x02')
                buf += int.to_bytes(int(lat * 1e7), length=4, byteorder='big', signed=True)
                buf += int.to_bytes(int(lon * 1e7), length=4, byteorder='big', signed=True)
                buf += int.to_bytes(spd, length=2, byteorder='big', signed=False)
                buf += int.to_bytes(hdg, length=2, byteorder='big', signed=False)
                buf += int.to_bytes(alt, length=2, byteorder='big', signed=False)
                buf += int.to_bytes(sats, length=1, byteorder='big', signed=False)
                buf += b'\x00'
                buf[-1] = crsf_frame_crc(buf)
                buf[1] = len(buf) - 2
                output(uart, buf)
                lastTime = time_us
    except FileNotFoundError:
        print(f"File not found: {fname}")
    except Exception as e:
        print(f"Error processing file: {e}")
    finally:
        is_processing = False
        save_gps_data()  # Ensure data is saved even if processing fails
        if uart:
            uart.close()

    if received_data:
        print("\nReceived UART data:")
        for data in received_data:
            print(data.hex())

class RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open('index.html', 'rb') as f:
                self.wfile.write(f.read())
        elif self.path == '/gps_data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(gps_data).encode())
        elif self.path == '/status':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({
                'isRunning': is_processing,
                'dataLength': len(gps_data)
            }).encode())
        elif self.path == '/airplane.svg':
            try:
                with open('airplane.svg', 'rb') as f:
                    self.send_response(200)
                    self.send_header('Content-type', 'image/svg+xml')
                    self.end_headers()
                    self.wfile.write(f.read())
            except FileNotFoundError:
                self.send_response(404)
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'airplane.png not found')
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/start':
            global gps_data, is_processing
            content_length = int(self.headers['Content-Length'])
            post_data = json.loads(self.rfile.read(content_length))
            gps_data = []
            save_gps_data()  # Clear saved data
            filename = post_data.get('filename', '')
            interval = float(post_data.get('interval', 0.5))
            port = post_data.get('port', None)
            baud = int(post_data.get('baud', 460800))
            jitter = float(post_data.get('jitter', 0.0))
            output_func = output_Msp if post_data.get('msp', False) else output_Crsf
            base_time_str = post_data.get('base_time', '2024-11-16T07:40:44.212029Z')
            base_time = None
            try:
                if 'T' in base_time_str and base_time_str.endswith('Z'):
                    base_time = datetime.datetime.fromisoformat(base_time_str.replace('Z', '+00:00'))
                else:
                    base_time = datetime.datetime.strptime(base_time_str, '%Y-%m-%d %H:%M:%S')
                    base_time = pytz.utc.localize(base_time)
            except (ValueError, TypeError):
                print("Invalid base time format, using default")
                base_time = datetime.datetime(2024, 11, 16, 7, 40, 44, 212029, tzinfo=pytz.UTC)
            if not filename:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'error': 'Filename is required'}).encode())
                return
            is_processing = True
            threading.Thread(target=processFile, args=(filename, interval, port, baud, jitter, output_func, base_time)).start()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'status': 'started'}).encode())
        else:
            self.send_response(404)
            self.end_headers()

def run_server():
    server_address = ('', 8000)
    httpd = HTTPServer(server_address, RequestHandler)
    print("Serving on http://localhost:8000")
   # webbrowser.open('http://localhost:8000')
    httpd.serve_forever()

if __name__ == '__main__':
    # Load existing gps_data on startup
    load_gps_data()
    run_server()