# coding: utf-8
# BME280, MCP3002(CdS), Motor, RGB LED を1つの Flask アプリに統合

import atexit
import threading
import time
from flask import Flask, request  # type: ignore
from werkzeug.exceptions import BadRequest  # type: ignore
import RPi.GPIO as GPIO  # type: ignore
import spidev  # type: ignore
from smbus2 import SMBus  # type: ignore

app = Flask(__name__)


def is_plain_object(value):
    return isinstance(value, dict)


def has_exactly_keys(obj, keys):
    return is_plain_object(obj) and set(obj.keys()) == set(keys)


def error_response(message, status=400):
    return {"error": message}, status


def parse_json_request():
    if not request.is_json:
        raise ValueError("Request body must be JSON")

    try:
        payload = request.get_json()
    except BadRequest:
        raise ValueError("Request body must be valid JSON")

    if not is_plain_object(payload):
        raise ValueError("JSON body must be an object")

    return payload


def validate_uint8(value, name):
    if type(value) is not int:
        raise ValueError(f"{name} must be an integer")
    if value < 0 or value > 255:
        raise ValueError(f"{name} must be from 0 to 255")
    return value


def validate_tear_command(payload):
    if not has_exactly_keys(payload, ("type", "params")):
        raise ValueError("Motor command must contain exactly type and params")
    if payload["type"] != "tear":
        raise ValueError('Motor command type must be "tear"')

    params = payload["params"]
    if not has_exactly_keys(params, ("speed", "duration")):
        raise ValueError("Motor params must contain exactly speed and duration")

    speed = validate_uint8(params["speed"], "speed")
    duration = validate_uint8(params["duration"], "duration")
    return speed, duration


def validate_led_command(payload):
    if not has_exactly_keys(payload, ("type", "params")):
        raise ValueError("LED command must contain exactly type and params")
    if payload["type"] != "led_change":
        raise ValueError('LED command type must be "led_change"')

    params = payload["params"]
    if not has_exactly_keys(params, ("color",)):
        raise ValueError("LED params must contain exactly color")

    color = params["color"]
    if not isinstance(color, str):
        raise ValueError("color must be a string")

    hex_color_to_pwm(color)
    return color.upper()


def channel_to_pwm(value):
    duty = round((value / 255.0) * 100, 1)
    if duty.is_integer():
        return int(duty)
    return duty


def hex_color_to_pwm(color):
    if not isinstance(color, str):
        raise ValueError("color must be a string")
    if len(color) != 7 or color[0] != "#":
        raise ValueError("color must be in #RRGGBB format")

    try:
        red_raw = int(color[1:3], 16)
        green_raw = int(color[3:5], 16)
        blue_raw = int(color[5:7], 16)
    except ValueError:
        raise ValueError("color must be in #RRGGBB format")

    return {
        "red": channel_to_pwm(red_raw),
        "green": channel_to_pwm(green_raw),
        "blue": channel_to_pwm(blue_raw),
    }

# ================================
# BME280
# ================================

bme_latest_sensor = {
    "temp": None,
    "pressure": None,
    "humidity": None,
    "timestamp": None,
}

bus_number = 1
i2c_address = 0x76
bus = SMBus(bus_number)

digT = []
digP = []
digH = []
t_fine = 0.0


def bme_write_reg(reg_address, data):
    bus.write_byte_data(i2c_address, reg_address, data)


def bme_get_calib_param():
    calib = []

    for i in range(0x88, 0x88 + 24):
        calib.append(bus.read_byte_data(i2c_address, i))
    calib.append(bus.read_byte_data(i2c_address, 0xA1))
    for i in range(0xE1, 0xE1 + 7):
        calib.append(bus.read_byte_data(i2c_address, i))

    digT.append((calib[1] << 8) | calib[0])
    digT.append((calib[3] << 8) | calib[2])
    digT.append((calib[5] << 8) | calib[4])
    digP.append((calib[7] << 8) | calib[6])
    digP.append((calib[9] << 8) | calib[8])
    digP.append((calib[11] << 8) | calib[10])
    digP.append((calib[13] << 8) | calib[12])
    digP.append((calib[15] << 8) | calib[14])
    digP.append((calib[17] << 8) | calib[16])
    digP.append((calib[19] << 8) | calib[18])
    digP.append((calib[21] << 8) | calib[20])
    digP.append((calib[23] << 8) | calib[22])
    digH.append(calib[24])
    digH.append((calib[26] << 8) | calib[25])
    digH.append(calib[27])
    digH.append((calib[28] << 4) | (0x0F & calib[29]))
    digH.append((calib[30] << 4) | ((calib[29] >> 4) & 0x0F))
    digH.append(calib[31])

    for i in range(1, 2):
        if digT[i] & 0x8000:
            digT[i] = (-digT[i] ^ 0xFFFF) + 1

    for i in range(1, 8):
        if digP[i] & 0x8000:
            digP[i] = (-digP[i] ^ 0xFFFF) + 1

    for i in range(0, 6):
        if digH[i] & 0x8000:
            digH[i] = (-digH[i] ^ 0xFFFF) + 1


def bme_compensate_p(adc_p):
    global t_fine

    v1 = (t_fine / 2.0) - 64000.0
    v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
    v2 = v2 + ((v1 * digP[4]) * 2.0)
    v2 = (v2 / 4.0) + (digP[3] * 65536.0)
    v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8) + ((digP[1] * v1) / 2.0)) / 262144
    v1 = ((32768 + v1) * digP[0]) / 32768

    if v1 == 0:
        return

    pressure = ((1048576 - adc_p) - (v2 / 4096)) * 3125
    if pressure < 0x80000000:
        pressure = (pressure * 2.0) / v1
    else:
        pressure = (pressure / v1) * 2

    v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0
    pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)

    bme_latest_sensor["pressure"] = pressure / 100


def bme_compensate_t(adc_t):
    global t_fine

    v1 = (adc_t / 16384.0 - digT[0] / 1024.0) * digT[1]
    v2 = (adc_t / 131072.0 - digT[0] / 8192.0) * (adc_t / 131072.0 - digT[0] / 8192.0) * digT[2]
    t_fine = v1 + v2

    temperature = t_fine / 5120.0
    bme_latest_sensor["temp"] = temperature
    bme_latest_sensor["timestamp"] = time.time()


def bme_compensate_h(adc_h):
    global t_fine

    var_h = t_fine - 76800.0

    if var_h == 0:
        return

    var_h = (adc_h - (digH[3] * 64.0 + digH[4] / 16384.0 * var_h)) * (
        digH[1]
        / 65536.0
        * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h))
    )

    var_h = var_h * (1.0 - digH[0] * var_h / 524288.0)

    if var_h > 100.0:
        var_h = 100.0
    elif var_h < 0.0:
        var_h = 0.0

    bme_latest_sensor["humidity"] = var_h


def bme_read_data():
    data = []
    for i in range(0xF7, 0xF7 + 8):
        data.append(bus.read_byte_data(i2c_address, i))

    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw = (data[6] << 8) | data[7]

    bme_compensate_t(temp_raw)
    bme_compensate_p(pres_raw)
    bme_compensate_h(hum_raw)


def bme_setup():
    osrs_t = 1
    osrs_p = 1
    osrs_h = 1
    mode = 3
    t_sb = 5
    bme_filter = 0
    spi3w_en = 0

    ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
    config_reg = (t_sb << 5) | (bme_filter << 2) | spi3w_en
    ctrl_hum_reg = osrs_h

    bme_write_reg(0xF2, ctrl_hum_reg)
    bme_write_reg(0xF4, ctrl_meas_reg)
    bme_write_reg(0xF5, config_reg)


def bme_worker():
    while True:
        bme_read_data()
        time.sleep(1)


@app.route("/bme280/sensor_data")
def bme280_sensor_data():
    return bme_latest_sensor


# ================================
# MCP3002 + CdS
# ================================

cds_latest_sensor = {
    "cds": None,
    "timestamp": None,
}

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 100000


def read_adc(channel: int) -> int:
    if channel not in (0, 1):
        raise ValueError("channel must be 0 or 1")

    if channel == 0:
        cmd = [0b01101000, 0x00]
    else:
        cmd = [0b01111000, 0x00]

    resp = spi.xfer2(cmd)
    return ((resp[0] & 0x03) << 8) | resp[1]


def read_cds():
    value = read_adc(0)
    cds_latest_sensor["cds"] = value
    cds_latest_sensor["timestamp"] = time.time()


def cds_worker():
    while True:
        read_cds()
        time.sleep(1)


@app.route("/cds/sensor_data")
def cds_sensor_data():
    return cds_latest_sensor


# ================================
# Motor (旧 mitsuki.py)
# ================================

AIN1 = 20
AIN2 = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)

pwm1 = GPIO.PWM(AIN1, 100)
pwm2 = GPIO.PWM(AIN2, 100)
pwm1.start(0)
pwm2.start(0)


def motor_forward(speed=70):
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(0)


def motor_stop():
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)


@app.route("/motor/command", methods=["POST"])
def handle_motor_command():
    try:
        payload = parse_json_request()
        speed, duration = validate_tear_command(payload)
    except ValueError as exc:
        motor_stop()
        return error_response(str(exc))

    duty = 40 + (speed / 255.0) * 60
    motor_forward(duty)
    time.sleep(duration)
    motor_stop()

    return {
        "status": "OK",
        "type": "tear",
        "speed": speed,
        "duty": round(duty, 1),
        "duration": duration,
    }


# ================================
# RGB LED
# ================================

R_PIN = 17
G_PIN = 27
B_PIN = 22

GPIO.setup(R_PIN, GPIO.OUT)
GPIO.setup(G_PIN, GPIO.OUT)
GPIO.setup(B_PIN, GPIO.OUT)

r = GPIO.PWM(R_PIN, 1000)
g = GPIO.PWM(G_PIN, 1000)
b = GPIO.PWM(B_PIN, 1000)

r.start(0)
g.start(0)
b.start(0)

current_color = [0.0, 0.0, 0.0]
target_color = [0.0, 0.0, 0.0]
color_lock = threading.Lock()

STEP = 2.0
UPDATE_INTERVAL = 0.03


def apply_pwm(red, green, blue):
    r.ChangeDutyCycle(red)
    g.ChangeDutyCycle(green)
    b.ChangeDutyCycle(blue)


def set_target_color(red, green, blue):
    with color_lock:
        target_color[0] = float(red)
        target_color[1] = float(green)
        target_color[2] = float(blue)


def fade_worker():
    global current_color

    while True:
        with color_lock:
            for i in range(3):
                diff = target_color[i] - current_color[i]
                if abs(diff) <= STEP:
                    current_color[i] = target_color[i]
                elif diff > 0:
                    current_color[i] += STEP
                else:
                    current_color[i] -= STEP

            red, green, blue = current_color

        apply_pwm(red, green, blue)
        time.sleep(UPDATE_INTERVAL)


@app.route("/led/command", methods=["POST"])
def handle_led_command():
    try:
        payload = parse_json_request()
        color = validate_led_command(payload)
        pwm = hex_color_to_pwm(color)
    except ValueError as exc:
        return error_response(str(exc))

    set_target_color(pwm["red"], pwm["green"], pwm["blue"])
    return {
        "status": "OK",
        "type": "led_change",
        "color": color,
        "pwm": pwm,
    }


@app.route("/")
def home():
    return {
        "service": "kokomi_raspi",
        "endpoints": [
            "/bme280/sensor_data",
            "/cds/sensor_data",
            "/motor/command",
            "/led/command",
        ],
    }


def cleanup():
    try:
        spi.close()
    except Exception:
        pass

    try:
        bus.close()
    except Exception:
        pass

    try:
        pwm1.stop()
        pwm2.stop()
    except Exception:
        pass

    try:
        r.stop()
        g.stop()
        b.stop()
    except Exception:
        pass

    GPIO.cleanup()


atexit.register(cleanup)


if __name__ == "__main__":
    bme_setup()
    bme_get_calib_param()

    threading.Thread(target=bme_worker, daemon=True).start()
    threading.Thread(target=cds_worker, daemon=True).start()
    threading.Thread(target=fade_worker, daemon=True).start()

    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False, threaded=True)
