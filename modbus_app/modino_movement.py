import time
import serial

def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def add_crc(data):
    crc = crc16(data)
    return data + bytes([crc & 0xFF, crc >> 8])


def read_registers(port):
    #     SLAVE_ID, FUNCTION_CODE, 0, START_REGISTER, 0, REGISTER_COUNT
    request = add_crc(bytes([1, 0x03, 0, 29, 0, 6]))
    port.reset_input_buffer()
    port.write(request)
    port.flush()

    # Read the response header (slave ID, function code, byte count)
    header = port.read(3)
    if len(header) != 3:
        raise RuntimeError("No Modbus response")

    if header[1] & 0x80:
        response = header + port.read(2)
        raise RuntimeError(f"Modbus exception {response[2]}")

    byte_count = header[2]
    response = header + port.read(byte_count + 2)
    if len(response) != 3 + byte_count + 2:
        raise RuntimeError("Incomplete Modbus response")
    if crc16(response) != 0:
        raise RuntimeError("Bad Modbus CRC")

    data = response[3:-2]
    return [(data[i] << 8) | data[i + 1] for i in range(0, len(data), 2)]


def main():
    print(f"Opening {"COM6"} at {"115200"} baud")
import serial
import time

# Assuming read_registers(port) is defined above

try:
    # 1. Open the port ONCE
    with serial.Serial("COM6", 115200, timeout=1) as port:
        print("Opening port and waiting for Arduino to reset...")
        time.sleep(2)  # Arduino resets when the serial port opens
        
        # 2. Start the continuous loop
        while True:
            regs = read_registers(port)
            
            # Optional but recommended: check if regs actually has data
            # to prevent an IndexError if a read fails or times out.
            if not regs or len(regs) < 6:
                continue 

            # 3. Parse the data
            movement_status = regs[0]
            movement_seq = regs[1]
            movement_millis = (regs[2] << 16) | regs[3]
            movement_roll_angle = regs[4]
            movement_pitch_angle = regs[5]
            
            # Adjust for signed 16-bit integers
            if movement_roll_angle > 32767:
                movement_roll_angle -= 65536
            if movement_pitch_angle > 32767:
                movement_pitch_angle -= 65536

            # 4. Print the data
            print(f"status: {movement_status}")
            print(f"seq: {movement_seq}")
            print(f"millis: {movement_millis}")
            print(f"roll: {movement_roll_angle} deg")
            print(f"pitch: {movement_pitch_angle} deg")
            print("-" * 20)  # Adds a visual separator between loops
            
            # 5. Add a small delay so you don't overwhelm your CPU or the serial buffer
            time.sleep(0.1) 

except KeyboardInterrupt:
    # Allows you to stop the loop gracefully by pressing Ctrl+C in the terminal
    print("\nProgram stopped by user.")
except serial.SerialException as e:
    print(f"Serial error: {e}")
    exit(1)
except Exception as e:
    print(f"Unexpected error: {e}")
    exit(1)

if __name__ == "__main__":
    main()
