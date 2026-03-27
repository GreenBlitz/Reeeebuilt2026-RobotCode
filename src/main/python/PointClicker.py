import sys
import time
import pyautogui
from NetworkTableManager import NetworkTableClient

CLIENT_NAME = "ClickPoint"
CLICK_POINT_TOPIC_NAME = "ClickPoint/ShouldClick"

CHECK_COOLDOWN_SECONDS = 0.1

# The screen coordinates to click — adjust as needed
CLICK_X = 0
CLICK_Y = 0


def perform_click(x: int, y: int) -> None:
    pyautogui.click(x, y)


def track_click_until_client_disconnect(client: NetworkTableClient) -> None:
    should_click = client.get_topic(CLICK_POINT_TOPIC_NAME).genericSubscribe()

    already_clicked = False
    while client.is_connected():
        if should_click.getBoolean(defaultValue=False):
            if not already_clicked:
                perform_click(CLICK_X, CLICK_Y)
                already_clicked = True
        else:
            already_clicked = False
        time.sleep(CHECK_COOLDOWN_SECONDS)


def run_click_point_client(ip: str) -> None:
    client = NetworkTableClient(ip, CLIENT_NAME)
    client.connect()
    track_click_until_client_disconnect(client)
    client.terminate()


if __name__ == '__main__':
    run_click_point_client(sys.argv[1])