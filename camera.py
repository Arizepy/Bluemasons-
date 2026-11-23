# #!/usr/bin/env python3
# import serial
# import struct
# import time
# import glob
# import subprocess
# import sys
# import os

# class CameraController:
#     def __init__(self, port=None, baud=115200, timeout=2.0):
#         self.port = port
#         self.baud = baud
#         self.timeout = timeout
#         self.ser = None
#         self._connected = False

#     def _disable_hupcl(self, port_path):
#         """Disable HUPCL on ev3dev/Linux to prevent ESP32 reset on open."""
#         try:
#             subprocess.run(["stty", "-F", port_path, "-hupcl"], 
#             check=True, capture_output=True)
#         except subprocess.CalledProcessError:
#             print(f"Warning: Could not disable HUPCL for {port_path}", file=sys.stderr)

#     def _autodetect_port(self):
#         """Find the first available USB serial port."""
#         candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
#         if not candidates:
#             raise RuntimeError("No USB serial device found. Is the ESP32 connected?")
#         return candidates[0]

#     def connect(self):
#         """Establish serial connection."""
#         if self._connected:
#             return

#         # Determine port
#         if not self.port:
#             self.port = self._autodetect_port()
        
#         print(f"Using port: {self.port}", flush=True)
        
#         # ev3dev specific: Disable HUPCL before opening
#         self._disable_hupcl(self.port)

#         try:
#             self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
#             self.ser.setDTR(False)
#             self.ser.setRTS(False)
#             time.sleep(2)  # Wait for ESP32 to stabilize
#             self._connected = True
#             print("Connected to ESP32.", flush=True)
#         except serial.SerialException as e:
#             raise RuntimeError(f"Failed to open {self.port}: {e}")

#     def disconnect(self):
#         """Close serial connection."""
#         if self.ser and self.ser.is_open:
#             self.ser.close()
#         self._connected = False
#         print("Disconnected.", flush=True)

#     def capture_image(self, filename="capture.jpg"):
#         """Capture an image from the camera and save it."""
#         if not self._connected:
#             self.connect()

#         # 1. Flush buffers
#         self.ser.reset_input_buffer()
#         time.sleep(0.1)
#         self.ser.reset_input_buffer()
        
#         # 2. Send Command
#         self.ser.write(b'S')
        
#         # 3. Scan for Header
#         header = b''
#         timeout = 5.0
#         start_time = time.time()
        
#         while time.time() - start_time < timeout:
#             if self.ser.in_waiting:
#                 byte = self.ser.read(1)
#                 header += byte
#                 if len(header) > 4:
#                     header = header[-4:]
#                 if header == b'IMG:':
#                     break
#             else:
#                 time.sleep(0.001)
#         else:
#             print(f"Error: Timeout waiting for header. Received: {header}", flush=True)
#             return False

#         # 4. Read Size
#         size_bytes = self.ser.read(4)
#         if len(size_bytes) < 4:
#             print("Error: Could not read image size.", flush=True)
#             return False
        
#         image_size = struct.unpack("<I", size_bytes)[0]
#         if image_size == 0 or image_size > 10_000_000:
#             print(f"Error: Invalid image size: {image_size}", flush=True)
#             return False

#         # 5. Read Data
#         data = bytearray()
#         remaining = image_size
#         while remaining > 0:
#             chunk_size = min(4096, remaining)
#             chunk = self.ser.read(chunk_size)
#             if not chunk:
#                 print("\nError: Connection lost during transfer.", flush=True)
#                 return False
#             data.extend(chunk)
#             remaining -= len(chunk)
#             print(f"\rReceiving: {100 - (remaining/image_size)*100:.1f}%", end='', flush=True)
        
#         print("\nCapture complete.", flush=True)
        
#         # 6. Save File
#         try:
#             with open(filename, "wb") as f:
#                 f.write(bytes(data))
#             print(f"Saved to {filename}", flush=True)
#             return True
#         except IOError as e:
#             print(f"Error saving file: {e}", flush=True)
#             return False

#     def send_command(self, cmd_char):
#         """Send a single character command (e.g., 'a'-'e')."""
#         if not self._connected:
#             self.connect()
        
#         cmd = cmd_char.lower().encode('utf-8')
#         self.ser.write(cmd)
#         print(f"Sent command: {cmd.decode()}", flush=True)
        
#         # Optional: Read text response
#         if self.ser.in_waiting:
#             response = self.ser.readline().decode('utf-8', errors='ignore').strip()
#             if response:
#                 print(f"Response: {response}", flush=True)
#                 return response
#         return None

#     def __enter__(self):
#         self.connect()
#         return self

#     def __exit__(self, exc_type, exc_val, exc_tb):
#         self.disconnect()

# # Example Usage for ev3dev
# if __name__ == "__main__":
#     try:
#         # Use context manager for automatic cleanup
#         with CameraController() as cam:
#             print("Commands: [S]nap, [A-E] Servo, [Q]uit")
            
#             while True:
#                 inp = input("> ").strip().lower()
#                 if inp == 'q':
#                     break
#                 elif inp == 's':
#                     cam.capture_image("/home/robot/capture.jpg")
#                 elif inp in ["a", "b", "c", "d", "e"]:
#                     cam.send_command(inp)
#                 else:
#                     print("Unknown command.", flush=True)
                    
#     except KeyboardInterrupt:
#         print("\nInterrupted by user.", flush=True)
#     except Exception as e:
#         print(f"Fatal Error: {e}", file=sys.stderr)
#         sys.exit(1)   

#!/usr/bin/env python3
import serial
import struct
import time
import glob
import subprocess
import sys
import threading

class CameraController:
    def __init__(self, port=None, baud=115200, timeout=2.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self._connected = False
        # Ensures the vision thread and robot loop don't collide on the serial line
        self.lock = threading.Lock() 

    def _disable_hupcl(self, port_path):
        """Disable HUPCL on ev3dev/Linux to prevent ESP32 reset on open."""
        try:
            # FIX: Replaced capture_output=True with Python 3.5 safe stdout/stderr pipes
            subprocess.run(["stty", "-F", port_path, "-hupcl"], 
                        check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except (subprocess.CalledProcessError, FileNotFoundError):
            # FIX: Replaced f-string with standard sys.stderr layout
            sys.stderr.write("Warning: Could not disable HUPCL for {}\n".format(port_path))
            sys.stderr.flush()

    def _autodetect_port(self):
        """Find the first available USB serial port."""
        candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        if not candidates:
            raise RuntimeError("No USB serial device found. Is the ESP32 connected?")
        return candidates[0]
    

    def connect(self):
        """Establish serial connection."""
        if self._connected:
            return

        if not self.port:
            self.port = self._autodetect_port()
        
        # FIX: Replaced f-string
        print("Using port: {}".format(self.port), flush=True)
        self._disable_hupcl(self.port)

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            time.sleep(2)  # Wait for ESP32 to stabilize
            self._connected = True
            print("Connected to ESP32.", flush=True)
        except serial.SerialException as e:
            # FIX: Replaced f-string
            raise RuntimeError("Failed to open {}: {}".format(self.port, e))

    def disconnect(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self._connected = False
        print("Disconnected.", flush=True)

    def get_image(self):
        """
        Thread-safe entry point required by TrayVision.
        Returns raw JPEG bytes on success, or None on failure.
        """
        with self.lock:
            return self._capture_raw_bytes()

    def _capture_raw_bytes(self):
        """Internal helper to stream byte chunks from ESP32 with zero loop lag."""
        if not self._connected:
            try:
                # Force port re-detection to ensure we catch where Linux moved the device
                self.port = None 
                self.connect()
            except Exception as e:
                print("[Camera Error] Reconnect failed: {}".format(e), flush=True)
                return None

        try:
            # Flush completely with a tighter 10ms safety window
            self.ser.reset_input_buffer()
            time.sleep(0.01)
            self.ser.reset_input_buffer()
            
            # Send Command character
            self.ser.write(b'S')
            
            # HIGH-SPEED BUFFERED BLOCK SCAN FOR HEADER
            buffer_window = b''
            timeout = 1.5  # Lowered timeout threshold to prevent long hanging blocks
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    # FIX: Read the entire waiting data block at once instead of self.ser.read(1)
                    chunk = self.ser.read(waiting)
                    buffer_window += chunk
                    
                    # Look for the absolute position index of the header marker
                    idx = buffer_window.find(b'IMG:')
                    if idx != -1:
                        # Slice away everything up to the end of the header marker
                        buffer_window = buffer_window[idx + 4:]
                        break
                        
                    # Guard memory footprint from infinite expansion if marker is missed
                    if len(buffer_window) > 1024:
                        buffer_window = buffer_window[-4:]
                else:
                    time.sleep(0.001)
            else:
                print("Error: Timeout waiting for header.", flush=True)
                return None

            # Read Size from the remaining read buffer stream pipeline
            while len(buffer_window) < 4:
                more_bytes = self.ser.read(4 - len(buffer_window))
                if not more_bytes:
                    print("Error: Could not read image size.", flush=True)
                    return None
                buffer_window += more_bytes
                
            size_bytes = buffer_window[:4]
            buffer_window = buffer_window[4:] # Keep leftover bytes for the image payload
            
            image_size = struct.unpack("<I", size_bytes)[0]
            if image_size == 0 or image_size > 10000000:
                print("Error: Invalid image size: {}".format(image_size), flush=True)
                return None

            # Read Data Block
            data = bytearray(buffer_window)  # Start with any residual image bytes we already read
            remaining = int(image_size) - len(data)
            
            while remaining > 0:
                chunk_size = min(4096, remaining)
                chunk = self.ser.read(chunk_size)
                if not chunk:
                    print("\nError: Connection lost during transfer.", flush=True)
                    return None
                data.extend(chunk)
                remaining -= len(chunk)
            
            print("[Camera] Raw JPEG stream capture complete {:d} bytes".format(len(data)), flush=True)
            return bytes(data)

        except (OSError, serial.SerialException) as e:
            print("\n[HARDWARE CRITICAL] Port dropped completely: {}".format(e), flush=True)
            self._connected = False
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            return None
    
    def send_command(self, cmd_char):
        """Send a single character command (e.g., 'a'-'e') thread-safely."""
        with self.lock:
            if not self._connected:
                self.connect()
            
            # Converts standard letters like 'a' into the byte data format b'a'
            cmd = str(cmd_char).lower().encode('utf-8')
            self.ser.write(cmd)
            print("Sent character command: {}".format(cmd.decode()), flush=True)
            
            if self.ser.in_waiting:
                response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print("Response from ESP32: {}".format(response), flush=True)
                    return response
            return None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
