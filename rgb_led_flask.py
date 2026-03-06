# led_flask_hex
from flask import Flask, request  # type: ignore
import RPi.GPIO as GPIO  # type: ignore

app = Flask(__name__)

R_PIN = 17
G_PIN = 27
B_PIN = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(R_PIN, GPIO.OUT)
GPIO.setup(G_PIN, GPIO.OUT)
GPIO.setup(B_PIN, GPIO.OUT)

r = GPIO.PWM(R_PIN, 1000)
g = GPIO.PWM(G_PIN, 1000)
b = GPIO.PWM(B_PIN, 1000)

r.start(0)
g.start(0)
b.start(0)


def set_color(red, green, blue):
    r.ChangeDutyCycle(red)
    g.ChangeDutyCycle(green)
    b.ChangeDutyCycle(blue)


def parse_lt_command(cmd: str):

    if not cmd.startswith("LT") or not cmd.endswith(";"):
        return None

    hexpart = cmd[2:-1]

    if len(hexpart) != 6:
        return None

    try:
        code = int(hexpart[0:2], 16)
    except ValueError:
        return None

    return code


def apply_emotion(code):

    if code == 0x01:       # 青：落ち着き
        set_color(0, 0, 100)

    elif code == 0x02:     # 赤：怒り
        set_color(100, 0, 0)

    elif code == 0x03:     # ピンク：喜び
        set_color(100, 0, 40)

    elif code == 0x04:     # 緑：正常
        set_color(0, 100, 0)

    elif code == 0x05:     # 紫：困惑
        set_color(60, 0, 100)

    else:
        set_color(0, 0, 0)


@app.route("/command", methods=["POST"])
def handle_command():

    command = request.get_data(as_text=True).strip()

    if not command:
        command = request.form.get("command", "").strip()

    print(f"受信: {command}")

    code = parse_lt_command(command)

    if code is not None:

        print(f"LED code=0x{code:02X}")

        apply_emotion(code)

        return f"OK: code={code}"

    else:
        print(f"不明コマンド: {command}")
        return f"Unknown command: {command}", 400


@app.route("/")
def home():
    return "LED control API running"


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5002)