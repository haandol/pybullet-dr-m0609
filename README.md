# Pybullet Doosan Robotics M0609 with Panda Gripper

This is a sample project to control Doosan Robotics M0609 with Panda Gripper using Pybullet.

You can control the robot by sending xyz position in terminal.

# Installation

copy `.env`

```bash
$ cp env/dev.env .env
```

install dependencies

```bash
$ pip install -r requirements.txt
```

# Usage

```python
$ python app.py
```

and then, you can send xyz position to control the robot.

```bash
Enter a position (x y z): 0.5 0.5 0.5
```
