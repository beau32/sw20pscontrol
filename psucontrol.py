import uasyncio as asyncio
import aioble
import bluetooth
import ure as re
from machine import Pin, PWM, UART, ADC
import time, esp32

# === BLE SETUP ===
UART_SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
UART_RX_UUID = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
UART_TX_UUID = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")

uart_service = aioble.Service(UART_SERVICE_UUID)

uart_rx_char = aioble.Characteristic(
    uart_service, UART_RX_UUID,
    write=True, write_no_response=True, capture=True
)

uart_tx_char = aioble.Characteristic(
    uart_service, UART_TX_UUID,
    notify=True
)

aioble.register_services(uart_service)

# === IO Setup ===
pwm = PWM(Pin(3, Pin.OUT))
pwm.freq(1000)

switchpin = Pin(0, Pin.OUT)
switchpin.value(0)

led = Pin(8, Pin.OUT)
led.value(0)

fan = Pin(10, Pin.OUT)
fan.value(0)

wl = Pin(1, Pin.OUT)
wl.value(0)

chk = Pin(2, Pin.IN, Pin.PULL_DOWN)  # CHK terminal input
bms_pin = Pin(4, Pin.IN, Pin.PULL_UP)  # BMS brush wear input pin
mth_pin = ADC(Pin(5))  # MTH voltage sense pin (ADC)
mth_pin.atten(ADC.ATTN_11DB)

# === CONFIG ===
UART_RX = 20
UART_TX = 21

duty = 0
on = False
device = ''
con = ''
gradual = False
pairing_mode = False  # If True, expect data from STM32 controller

uart = UART(1, baudrate=9600, tx=Pin(UART_TX), rx=Pin(UART_RX))

VALID_SS = {0, 1}
SPD_MIN, SPD_MAX = 0, 255
BMS_MIN, BMS_MAX = 11.0, 14.8
MTH_MIN, MTH_MAX = 10.0, 14.0
SPD_CUTOFF = 80

# === FAULT DETECTION ===
fault_codes = set()
MTH_LOW_LIMIT = 10.0
MTH_HIGH_LIMIT = 14.5
MOTOR_CURRENT_LIMIT = 60.0
MTH_FAULT_DELAY = 2.0
RELAY_STUCK_DELAY = 2.0

# === Logging Helper ===
def log(level: str, message: str):
    text = f"[{level}] {message}"
    print(text)
    try:
        uart_tx_char.notify(con, text)
    except:
        pass

# === Sensor Functions ===
def get_motor_current(mth_voltage: float = None):
    if mth_voltage is not None:
        return mth_voltage
    raw = mth_pin.read()
    voltage = raw * 3.6 / 4095
    return voltage

def get_bms_state():
    return bms_pin.value()

# === Indicator Functions ===
def set_lamps(state: int):
    wl.value(state)
    led.value(state)

def blink(times, duration=0.25, gap=0.25):
    for _ in range(times):
        set_lamps(1)
        time.sleep(duration)
        set_lamps(0)
        time.sleep(gap)

def blink_code(code):
    tens = code // 10
    ones = code % 10
    blink(tens, duration=0.5, gap=0.5)
    time.sleep(0.5)
    blink(ones, duration=0.25, gap=0.25)
    time.sleep(1.5)

async def diagnostic_mode():
    log("INFO", "Diagnostic mode active (CHK grounded)")
    set_lamps(0)
    while chk.value() == 0:
        for code in sorted(fault_codes):
            log("FAULT", f"Blinking code {code}")
            blink_code(code)
        time.sleep(2)
    set_lamps(0)
    log("INFO", "Diagnostic mode exited (CHK released)")

# === Fault Logic ===
last_mth_zero = None
last_relay_stuck = None

def update_faults(mth_voltage, motor_commanded):
    global last_mth_zero, last_relay_stuck
    motor_current = get_motor_current(mth_voltage)
    bms_state = get_bms_state()
    now = time.ticks_ms()

    # Code 11: Motor voltage 0V
    if motor_current == 0.0:
        if last_mth_zero is None:
            last_mth_zero = now
        elif time.ticks_diff(now, last_mth_zero) >= int(MTH_FAULT_DELAY * 1000):
            fault_codes.add(11)
            log("FAULT", "Code 11 triggered (MTH = 0V)")
    else:
        last_mth_zero = None
        fault_codes.discard(11)

    # Code 12: Relay stuck ON
    if not motor_commanded and motor_current > 1.0:
        if last_relay_stuck is None:
            last_relay_stuck = now
        elif time.ticks_diff(now, last_relay_stuck) >= int(RELAY_STUCK_DELAY * 1000):
            fault_codes.add(12)
            log("FAULT", "Code 12 triggered (Relay stuck ON)")
    else:
        last_relay_stuck = None
        fault_codes.discard(12)

    # Code 13: MTH out of range
    if motor_current < MTH_LOW_LIMIT or motor_current > MTH_HIGH_LIMIT:
        fault_codes.add(13)
        log("FAULT", "Code 13 triggered (MTH out of range)")
    else:
        fault_codes.discard(13)

    # Code 21: Overvoltage
    if motor_current > 14.2:
        fault_codes.add(21)
        log("FAULT", "Code 21 triggered (Overvoltage)")
    else:
        fault_codes.discard(21)

    # Code 22: Overcurrent
    if motor_current > MOTOR_CURRENT_LIMIT:
        fault_codes.add(22)
        log("FAULT", "Code 22 triggered (Overcurrent)")
    else:
        fault_codes.discard(22)

    # Code 31: Brush wear
    if bms_state == 0:
        fault_codes.add(31)
        log("FAULT", "Code 31 triggered (Brush wear)")
    else:
        fault_codes.discard(31)

# === Control Functions ===
def set_potentiometer(percent):
    global duty, gradual
    current = duty
    max_duty = 1023
    delay = 0.01
    STEP = int(max_duty * 0.5 / 100)
    percent = max(0, min(percent, 100))
    duty = int(percent * 1023 / 100)

    if gradual:
        if duty > current:
            for d in range(current, duty + 1, STEP):
                pwm.duty(max_duty - d)
                time.sleep(delay)
        else:
            for d in range(current, duty - 1, -STEP):
                pwm.duty(max_duty - d)
                time.sleep(delay)
    else:
        pwm.duty(max_duty - duty)

def parse_uart_line(line):
    try:
        line = line.strip()
        parts = line.split(',')
        if len(parts) != 6:
            return None
        ss1, ss2, spd, ef1, bms, mth = map(int, parts)
        return {
            'ss1': ss1,
            'ss2': ss2,
            'spd': spd,
            'ef1': ef1,
            'bms': bms,
            'mth': mth
        }
    except Exception as e:
        log("ERROR", f"UART parse failed: {e}")
        return None

def is_safe(data):
    if data['ss1'] not in VALID_SS or data['ss2'] not in VALID_SS:
        return False
    if not (BMS_MIN <= data['bms'] <= BMS_MAX):
        return False
    if not (MTH_MIN <= data['mth'] <= MTH_MAX):
        return False
    return True

# === Tasks ===
async def input_task():
    global duty, on, device, con, led, gradual, pairing_mode
    uarton = False

    while True:
        if chk.value() == 0:
            await diagnostic_mode()

        conn, data = await uart_rx_char.written()
        temp = esp32.mcu_temperature()

        if pairing_mode and uart.any() and uarton:
            line = uart.readline()
            if line:
                try:
                    decoded = line.decode('utf-8')
                    data = parse_uart_line(decoded)
                except Exception as e:
                    log("ERROR", f"UART decode error: {e}")
                    data = None

        if not pairing_mode:
            mth_local = int(get_motor_current() * 10)
            data = {
                'ss1': 1,
                'ss2': 1,
                'spd': 0,
                'ef1': 0,
                'bms': int(get_bms_state()*10),
                'mth': mth_local
            }

        if data:
            mth_voltage = float(data['mth'])
            motor_commanded = on
            update_faults(mth_voltage, motor_commanded)

            safe = is_safe(data)
            if safe and data['spd'] <= SPD_CUTOFF:
                wl.off()
                switchpin.on()
                pwm_val = data['spd'] / 255 * 100
                set_potentiometer(int(pwm_val))
                log("DATA", f"OK {data}, PWM: {pwm_val}")
            else:
                wl.on()
                switchpin.off()
                set_potentiometer(0)
                log("WARN", f"Motor disabled: fault or high speed {data}")
        else:
            wl.on()
            switchpin.off()
            pwm.duty_u16(0)

        if isinstance(data, bytes) and len(data) > 0:
            try:
                text = data.decode("utf-8").strip()
            except UnicodeDecodeError:
                text = "(non-UTF8 data)"
        else:
            text = "(empty or invalid data)"

        if text == 'switch on':
            switchpin.value(1)
            on = True
            log("INFO", "Switch ON")
        elif text == 'switch off':
            switchpin.value(0)
            on = False
            log("INFO", "Switch OFF")

        if text == 'gradual on':
            gradual = True
            log("INFO", "Gradual mode ON")
        elif text == 'gradual off':
            gradual = False
            log("INFO", "Gradual mode OFF")

        if text == 'pairing on':
            pairing_mode = True
            log("PAIRING", "STM32 pairing mode enabled")
        elif text == 'pairing off':
            pairing_mode = False
            log("PAIRING", "STM32 pairing mode disabled")

        if text == 'info':
            log("INFO", f'PWM: {duty} ON: {on} PAIR: {pairing_mode} TP: {temp}C')

        if text == 'uart on':
            uarton = True
            log("INFO", f'UART ON - PWM: {duty} ON: {on} BL: {device} TP: {temp}C')
        else:
            uarton = False

        match = re.match("pwm (.*)", text)
        if match:
            value = match.group(1)
            set_potentiometer(int(value))
            log("INFO", f"PWM set to {value}")

async def peripheral_task():
    global device, con, led
    while True:
        try:
            # Timeout after 30 seconds if no connection
            connection = await asyncio.wait_for(
                aioble.advertise(
                    250_000,
                    name="PicoUART",
                    services=[UART_SERVICE_UUID],
                ),
                30
            )
            async with connection:
                led.value(1)
                device = connection.device
                con = connection
                log("INFO", f"Connected to {device}")
                await connection.disconnected(timeout_ms=None)
                led.value(0)
                log("INFO", "Disconnected, restarting advertising")
        except asyncio.TimeoutError:
            # Retry after timeout to prevent blocking
            log("WARN", "BLE advertising timeout, retrying...")
            led.value(0)
            await asyncio.sleep(1)

async def fan_operation():
    global fan
    temp = esp32.mcu_temperature()
    if temp > 35:
        fan.value(1)
    else:
        fan.value(0)

# === MAIN ===
async def main():
    try:
        t1 = asyncio.create_task(peripheral_task())
        #t2 = asyncio.create_task(input_task())
        #t3 = asyncio.create_task(fan_operation())
        await asyncio.gather(t1) #, t2, t3)
    except Exception as e:
        log("ERROR", f"Notify failed: {e}")

asyncio.run(main())
