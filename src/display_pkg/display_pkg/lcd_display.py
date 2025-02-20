from smbus2 import SMBus
import subprocess
import time

class I2C_Device():
    def __init__(self, bus: int, address: int) -> None:
        self.bus = bus
        self.address = address

        # Commadns for the LCD
        self.LCD_CHR = 0b00000001  # Mode - Sending data
        self.LCD_CMD = 0b00000000  # Mode - Sending command
        self.LCD_BACKLIGHT = 0x08  # Backlight ON
        self.ENABLE = 0b00000100   # Enable bit
        return None
    
    def lcd_byte(self, data: int, mode: int) -> None:
        high_nibble = mode | (data & 0xF0) | self.LCD_BACKLIGHT
        low_nibble = mode | ((data << 4) & 0xF0) | self.LCD_BACKLIGHT
        self.bus.write_byte(self.address, high_nibble)
        self.lcd_toggle_enable(high_nibble)
        self.bus.write_byte(self.address, low_nibble)
        self.lcd_toggle_enable(low_nibble)
        return None

    def lcd_toggle_enable(self, data: int) -> None:
        time.sleep(0.0005)
        self.bus.write_byte(self.address, data | self.ENABLE)
        time.sleep(0.0005)
        self.bus.write_byte(self.address, data & ~self.ENABLE)
        time.sleep(0.0005)
        return None
    
    def lcd_init(self) -> None:
        self.lcd_byte(0x33, self.LCD_CMD)  # 110011 Initialize
        self.lcd_byte(0x32, self.LCD_CMD)  # 110010 Initialize
        self.lcd_byte(0x06, self.LCD_CMD)  # Cursor move direction
        self.lcd_byte(0x0C, self.LCD_CMD)  # Turn Cursor Off
        self.lcd_byte(0x28, self.LCD_CMD)  # 2 Line Display
        self.lcd_byte(0x01, self.LCD_CMD)  # Clear Display
        time.sleep(0.005)
        return None
    
    def lcd_string(self, message: str, line: int) -> None:
        if line == 1:
            self.lcd_byte(0x80, self.LCD_CMD)
        elif line == 2:
            self.lcd_byte(0xC0, self.LCD_CMD)
        
        print(f"Message: {message}")
        print(f"Message Length: {len(message)}")
        time.sleep(0.005)
        self.lcd_byte(0x01, self.LCD_CMD)
        time.sleep(0.005)

        if len(message) <= 16:
            for char in message:
                self.lcd_byte(ord(char), self.LCD_CHR)
        else:
            num_chars = len(message) - 15
            for char in message[0:16]:
                print(f"Char: {char}")
                self.lcd_byte(ord(char), self.LCD_CHR)
            time.sleep(0.5)

            for num in range(1, num_chars):
                self.lcd_byte(0x01, self.LCD_CMD)
                time.sleep(0.005)
                if line == 1:
                    self.lcd_byte(0x80, self.LCD_CMD)
                elif line == 2:
                    self.lcd_byte(0xC0, self.LCD_CMD)

                time.sleep(0.005)
                for char in message[num:16 + num]:
                    print(f"Char: {char}")
                    self.lcd_byte(ord(char), self.LCD_CHR)
                time.sleep(0.5)
        return None

def get_ssid_ip():
    ip = list()
    try:
        ssid = subprocess.check_output(["iwgetid", "-r"]).decode("utf-8")
        ip   = subprocess.check_output(["hostname", "-I"]).decode("utf-8")
        ip = ip.strip('\n ').split(" ")
    except subprocess.CalledProcessError as err:
        ssid = "**NO CONNECTION**"

    return (ssid.strip(), ip)

def main() -> int:
    
    # Initialize I2C Bus
    I2C_BUS = 7
    LCD_ADDRESS = 0x27

    # Create Node
    node = I2C_Device( SMBus(I2C_BUS), LCD_ADDRESS )
    
    # Initialize Display
    node.lcd_init()

    # Send Message
    # node.lcd_string("Hello World!", 1)
    # node.lcd_string("Start from cron", 2)
    
    # node.lcd_string("0123456789ABCDEFGHIJKLMNOP", 1)

    try:
        while True:
            (ssid, ips) = get_ssid_ip()
            node.lcd_string(f"SSID: {ssid}", 1)
            time.sleep(1.0)
            for ip in ips:
                node.lcd_string(f"IP: {ip}", 1)
                time.sleep(1.0)
                
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    return 0

if __name__ == "__main__":
    exit( main() )