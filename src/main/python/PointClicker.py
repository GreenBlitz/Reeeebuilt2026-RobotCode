import sys
import time
import pyautogui
from NetworkTableManager import NetworkTableClient

CLIENT_NAME = "ClickPoint"
CLICK_POINT_TOPIC_NAME = "ClickPoint/ShouldClick"

CHECK_COOLDOWN_SECONDS = 0.1
CLICK_X = 2560 // 2
CLICK_Y = 1664 // 2

def perform_click(x: int, y: int) -> None:
    print(f"clicking at {x}, {y}", flush=True)
    pyautogui.click(x, y)

def track_click_until_client_disconnect(client: NetworkTableClient) -> None:
    should_click = client.get_topic(CLICK_POINT_TOPIC_NAME).genericSubscribe()

    already_clicked = False
    while True:
        connected = client.is_connected()
        print("connected:", connected, flush=True)

        if connected:
            value = should_click.getBoolean(defaultValue=False)
            print("topic value:", value, flush=True)

            if value:
                if not already_clicked:
                    perform_click(CLICK_X, CLICK_Y)
                    already_clicked = True
            else:
                already_clicked = False

        time.sleep(CHECK_COOLDOWN_SECONDS)

def run_click_point_client(ip: str) -> None:
    print("starting client", flush=True)
    print("ip:", ip, flush=True)

    client = NetworkTableClient(ip, CLIENT_NAME)
    client.connect()

    track_click_until_client_disconnect(client)
    client.terminate()

if __name__ == '__main__':
    print("hello", flush=True)
    print("argv:", sys.argv, flush=True)
    run_click_point_client(sys.argv[1])