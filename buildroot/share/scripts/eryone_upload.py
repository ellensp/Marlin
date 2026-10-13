#
# eryone_upload.py
# Upload firmware over USB serial to the Eryone ERY32 Mini (Eryone ER20) serial bootloader.
#
# This replaces Eryone's "FlashTool" GUI. The bootloader listens on USART2 (the USB serial
# port) at 115200 baud and waits ~1 second after reset for a 'C' before starting the app.
# Once it has the 'C' it shows a menu; choosing '1' erases the application area and
# receives the firmware using YMODEM.
#
# If Marlin is running, 'M997' is sent to reboot the board into the bootloader, so no
# button press is needed. Otherwise press the reset button on the printer when asked.
#
# Use with PlatformIO:
#   extra_scripts   = ${...extra_scripts} pre:buildroot/share/scripts/eryone_upload.py
#   upload_protocol = custom
#
# Or standalone:
#   python3 buildroot/share/scripts/eryone_upload.py [port] <firmware.bin>
#
import os, sys, time

BAUDRATE = 115200

# CH340 USB serial bridge used on the ERY32 Mini
CH340_VID, CH340_PID = 0x1A86, 0x7523

# Application area erased by the bootloader before receiving (see bootloader common.h)
#   bootloader1118.bin : 400000 bytes rounded up to 2K pages
#   bootloader_old.bin : 250000 bytes rounded up to 2K pages
ERASE_SIZE_1118 = -(-400000 // 2048) * 2048
ERASE_SIZE_OLD  = -(-250000 // 2048) * 2048

# YMODEM control bytes
SOH, STX, EOT, ACK, NAK, CA, CRC16 = 0x01, 0x02, 0x04, 0x06, 0x15, 0x18, 0x43

class UploadError(Exception): pass

def _crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        crc &= 0xFFFF
    return crc

def _packet(num, data):
    # The bootloader frames packets by inter-character timeout (~170µs), so each
    # packet must be written in one piece.
    head = STX if len(data) == 1024 else SOH
    return bytes((head, num & 0xFF, ~num & 0xFF)) + data + _crc16(data).to_bytes(2, 'big')

def upload(port_name, firmware_path, log=print):
    import serial

    if not os.path.isfile(firmware_path):
        raise UploadError(f"Firmware file '{firmware_path}' not found. Build the firmware first.")
    with open(firmware_path, 'rb') as f: firmware = f.read()
    if not firmware: raise UploadError(f"'{firmware_path}' is empty")

    padded_size = -(-len(firmware) // 1024) * 1024
    if padded_size > ERASE_SIZE_1118:
        raise UploadError(f'Firmware is {len(firmware)} bytes. The Eryone bootloader only erases {ERASE_SIZE_1118} bytes.')
    if padded_size > ERASE_SIZE_OLD:
        log(f'WARNING: Firmware is {len(firmware)} bytes. Boards with the older Eryone bootloader only erase {ERASE_SIZE_OLD} bytes.')

    try:
        port = serial.Serial(port_name, baudrate=BAUDRATE, timeout=0, write_timeout=5)
    except serial.SerialException as e:
        msg = str(e).lower()
        if 'no such file' in msg or 'cannot find' in msg:
            hint = 'The port does not exist. Is the printer connected by USB and powered on?'
        elif 'busy' in msg or (os.name == 'nt' and 'denied' in msg):
            hint = 'The port is in use. Close any other program using it (serial monitor, OctoPrint, Pronterface...).'
        elif 'permission' in msg:
            hint = "Permission denied. Add your user to the 'dialout' group (sudo usermod -a -G dialout $USER), then log out and back in."
        else:
            hint = str(e)
        raise UploadError(f"Could not open '{port_name}'. {hint}")

    rx = bytearray()    # Received bytes not yet consumed

    def read_until(want, timeout):
        # Wait for any byte in 'want' (bytes) or a substring (str), consuming input up to
        # and including the match. Returns the matched byte or string, or None on timeout.
        deadline = time.time() + timeout
        while True:
            if isinstance(want, str):
                i = rx.find(want.encode())
                if i >= 0:
                    del rx[:i + len(want)]
                    return want
            else:
                i = next((n for n, b in enumerate(rx) if b in want), -1)
                if i >= 0:
                    b = rx[i]
                    del rx[:i + 1]
                    return b
                rx.clear()
            if time.time() >= deadline: return None
            chunk = port.read(256)
            if chunk: rx.extend(chunk)
            else: time.sleep(0.005)

    def send_packet(num, data, ack_timeout):
        pkt = _packet(num, data)
        for _ in range(5):
            port.write(pkt); port.flush()
            r = read_until(bytes((ACK, NAK, CA)), ack_timeout)
            if r == ACK: return
            if r == CA: raise UploadError(f'Bootloader aborted the transfer at packet {num}')
        raise UploadError(f'No acknowledgement for packet {num}')

    try:
        #
        # Enter the bootloader
        #
        port.reset_input_buffer()
        log('Sending M997 to reboot into the bootloader...')
        port.write(b'\nM997\n'); port.flush()

        # Keep sending 'C' until the bootloader menu appears
        buf, start, prompted = b'', time.time(), False
        while b'please choose' not in buf:
            if time.time() - start > 60:
                raise UploadError('Timed out waiting for the bootloader menu')
            if not prompted and time.time() - start > 3:
                log('No response. Press the reset button on the printer (under the LCD knob).')
                prompted = True
            port.write(b'C')
            time.sleep(0.02)
            buf = (buf + port.read(256))[-256:]

        # Let the trailing 'C's drain before choosing "1. Update program"
        time.sleep(0.3)
        port.reset_input_buffer()
        port.write(b'1'); port.flush()
        if not read_until('Please select to send the file', 3):
            raise UploadError("Bootloader did not accept the 'Update program' option")

        #
        # YMODEM transfer
        #
        if read_until(bytes((CRC16,)), 5) is None:
            raise UploadError('Bootloader did not request the file')

        log('Erasing...')
        name = os.path.basename(firmware_path).encode()[:100]
        header = (name + b'\0' + str(len(firmware)).encode() + b'\0').ljust(128, b'\0')
        send_packet(0, header, 30)      # The bootloader erases flash before acknowledging
        if read_until(bytes((CRC16,)), 3) is None:
            raise UploadError('Bootloader did not start the data transfer')

        total = -(-len(firmware) // 1024)
        for i in range(total):
            send_packet(i + 1, firmware[i * 1024:(i + 1) * 1024].ljust(1024, b'\xFF'), 3)
            sys.stdout.write(f'\rWriting... {100 * (i + 1) // total}%'); sys.stdout.flush()
        sys.stdout.write('\n')

        # End of file: the first EOT is NAKed, the second is ACKed
        for _ in range(3):
            port.write(bytes((EOT,))); port.flush()
            if read_until(bytes((ACK, NAK)), 3) == ACK: break
        else:
            raise UploadError('Bootloader did not acknowledge the end of the file')

        # End of batch: an empty header block
        if read_until(bytes((CRC16,)), 3) is None:
            raise UploadError('Bootloader did not request the end of batch')
        port.write(_packet(0, bytes(128))); port.flush()
        read_until('to app', 5)
        log('Firmware update completed')
    except serial.SerialException as e:
        raise UploadError(f"Lost connection to '{port_name}' ({e}). Check the USB cable, then upload again.")
    finally:
        try:
            port.close()
        except Exception:
            pass

def find_port():
    # Prefer the board's CH340, otherwise the only USB serial port present
    from serial.tools import list_ports
    usb = [ p for p in list_ports.comports() if p.vid is not None ]
    ch340 = [ p.device for p in usb if p.vid == CH340_VID and p.pid == CH340_PID ]
    if len(ch340) == 1: return ch340[0]
    if len(usb) == 1: return usb[0].device
    if not usb:
        raise UploadError('No USB serial port found. Is the printer connected by USB and powered on?')
    raise UploadError('More than one USB serial port found ('
                      + ', '.join(p.device for p in usb)
                      + "). Set 'upload_port' to the printer's port.")

def main(port_name, firmware):
    if not port_name: port_name = find_port()
    print(f"Uploading '{os.path.basename(firmware)}' to '{port_name}'")
    upload(port_name, firmware)

if __name__ == '__main__':
    if len(sys.argv) not in (2, 3):
        print(f'Usage: {sys.argv[0]} [port] <firmware.bin>')
        sys.exit(2)
    try:
        main(sys.argv[1] if len(sys.argv) == 3 else None, sys.argv[-1])
    except UploadError as e:
        print(f'ERROR: {e}')
        sys.exit(1)
else:
    from SCons.Script import DefaultEnvironment
    env = DefaultEnvironment()

    #-----------------#
    # Upload Callback #
    #-----------------#
    def Upload(source, target, env):
        firmware = os.path.join(env.subst('$BUILD_DIR'), env.subst('${PROGNAME}.bin'))
        try:
            main(env.subst('$UPLOAD_PORT'), firmware)
        except UploadError as e:
            # Report the problem without a traceback; a non-zero result fails the upload
            print(f'\nERROR: {e}\n')
            return 1
        return 0

    # Attach custom upload callback
    env.Replace(UPLOADCMD=Upload)
