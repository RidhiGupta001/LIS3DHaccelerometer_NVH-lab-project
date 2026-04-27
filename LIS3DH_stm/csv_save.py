import serial
import time

# --- CONFIGURATION ---
PORT = 'COM4'  # Change to your port
BAUD_RATE = 921600
FILE_NAME = f"canbeam300_harmonic_{int(time.time())}.csv"

def start_logger():
    try:
        # Open serial with a larger internal buffer
        ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
        ser.set_buffer_size(rx_size=128000, tx_size=128000)
        
        time.sleep(2)  # Wait for ESP32 reset
        ser.reset_input_buffer() # Clear any junk data from startup
        
        print(f"Connected to {PORT} at {BAUD_RATE} baud.")
        print(f"Recording to: {FILE_NAME}")
        print("Press Ctrl+C to stop.")

        with open(FILE_NAME, mode='w', encoding='utf-8', buffering=1) as file:
            while True:
                if ser.in_waiting > 0:
                    # Read everything currently in the buffer
                    # Using errors='ignore' prevents crashes on partial characters
                    data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    
                    if data:
                        file.write(data)
                        # We don't print every line to the terminal anymore 
                        # because printing to terminal is SLOW and causes lag.
                
    except KeyboardInterrupt:
        print("\nRecording stopped safely.")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    start_logger()