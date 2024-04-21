import os
import sys
import signal
import logging
import multiprocessing as mp

import dotenv
import pybullet as p
from rich.logging import RichHandler

from libs.robot import RobotFactory
from libs.env import Environment
from libs.messages.commands import MoveToXYZ
from libs.messages.events import EnvironmentReady, RobotMoved

dotenv.load_dotenv()

logging.basicConfig(
    level=logging.INFO, format="%(message)s", datefmt="[%X]", handlers=[RichHandler()]
)
logger = logging.getLogger("app")


def get_position_from_user_input():
    position = None
    while position is None or len(position) != 3:
        user_input = input(">>> Enter a xyz positions (x, y, z):")
        position = user_input.split(",")
        if len(position) != 3:
            logger.error(
                "Invalid input. Please enter a valid xyz position. Try again..."
            )
    return [float(p) for p in position]


def disconnect(sig, frame):
    logger.info("Disconnecting...")
    p.disconnect()
    logger.info("Disconnected.")
    sys.exit(0)


signal.signal(signal.SIGINT, disconnect)


if __name__ == "__main__":
    # create shared memory
    manager = mp.Manager()
    cmd_q = manager.Queue()
    evt_q = manager.Queue()

    # create robot and environment
    robot = RobotFactory.create(os.environ.get("ROBOT_NAME", "franka"))
    env = Environment(robot, cmd_q, evt_q)
    env.start()

    # wait for environment to be ready
    while True:
        evt = evt_q.get()
        logger.info(f"[EVT] {evt}")
        if isinstance(evt, EnvironmentReady) or isinstance(evt, RobotMoved):
            position = get_position_from_user_input()
            cmd_q.put(MoveToXYZ(position=position))
        else:
            logger.error("Unexpected event received: {evt}.\nExiting...")
            break

    env.terminate()
