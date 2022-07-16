#!/usr/bin/env python
# ********************************************************************************
# * Copyright (C) 2017-2020 German Aerospace Center (DLR).
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0
# *
# * Contributors:
# *   Thomas Lobig
# ********************************************************************************
import sys
import tty
try:
    import rospy
    from std_msgs.msg import Bool
except:
    print("rospy not found, need to source ROS to run this")
    sys.exit(-1)
import termios
import asyncio
import json
try:
    import evdev
except:
    print("evdev not found - consider installing with: pip3 install evdev")
    sys.exit(-1)

# A simple publisher to send a signal on a specific keystroke


class usb_config:
    def __init__(self):
        self.device = ""
        self.keymap = {}
        self.topic = "keyboard"


class Publisher(object):
    def __init__(self):
        self._topic_sup_lc = 'FUN/LangeChangeSuppresion'
        self._topic_flcl = 'FUN/ForceLanechange/left'
        self._topic_flcr = 'FUN/ForceLanechange/right'
        self._topic_slow = 'FUN/ForceSlowManeuvers'
        self._publisher_sup_lc = rospy.Publisher(
            self._topic_sup_lc,  Bool, queue_size=1)
        self._publisher_flcl = rospy.Publisher(
            self._topic_flcl, Bool, queue_size=1)
        self._publisher_flcr = rospy.Publisher(
            self._topic_flcr, Bool, queue_size=1)
        self._publisher_slow = rospy.Publisher(
            self._topic_slow, Bool, queue_size=1)
        self.reader = None

    async def prepare_pipeline(self):
        self.loop = asyncio.get_event_loop()
        self.reader = asyncio.StreamReader()
        self.protocol = asyncio.StreamReaderProtocol(self.reader)
        await self.loop.connect_read_pipe(lambda: self.protocol, sys.stdin)

    async def update(self):

        pressed_key = await self.reader.read(1)
        pressed_key = pressed_key.decode()
        if(pressed_key in ['w', 'W']):
            self.send_msg("suppress")

        elif(pressed_key in ['x', 'X']):
            self.send_msg("suppress")

        elif(pressed_key in ['s', 'S']):
            self.send_msg("slow")

        elif(pressed_key in ['a', 'A']):
            self.send_msg("llc_only")

        elif(pressed_key in ['d', 'D']):
            self.send_msg("rlc_only")

        elif(pressed_key == chr(27)):
            rospy.signal_shutdown("Escape key pressed")

    def send_msg(self, topic_name):

        if topic_name == "suppress":
            print("supressing lanechanges")
            self._publisher_sup_lc.publish(Bool(True))

        elif topic_name == "llc_only":
            print("forcing left lanechanges only")
            self._publisher_flcl.publish(Bool(True))

        elif topic_name == "rlc_only":
            print("forcing right lanechanges only")
            self._publisher_flcr.publish(Bool(True))

        elif topic_name == "slow":
            print("forcing slow maneuvers only")
            self._publisher_slow.publish(Bool(True))


class ros_wrapper:
    def __init__(self, config: usb_config, pub: Publisher):
        self.device = evdev.InputDevice(config.device)
        self.config = config
        self.pub = pub

    async def get_key_on_down_or_up(self):
        print("... listening for keypresses on device " + self.config.device)
        while not rospy.is_shutdown():
            try:
                async for event in self.device.async_read_loop():
                    if event.type == 1:
                        keycode = evdev.categorize(event).keycode
                        if isinstance(keycode, list):
                            keycode = keycode[0]
                        if keycode in self.config.keymap:
                            if event.value == 0:
                                if "up" in self.config.keymap[keycode]:
                                    self.pub.send_msg(
                                        self.config.keymap[keycode]["up"])

                            elif event.value == 1:
                                if "down" in self.config.keymap[keycode]:
                                    self.pub.send_msg(
                                        self.config.keymap[keycode]["down"])

                            elif event.value == 2:
                                if "hold" in self.config.keymap[keycode]:
                                    self.pub.send_msg(
                                        self.config.keymap[keycode]["hold"])

                    if rospy.is_shutdown():
                        return
            except:
                print("lost connection to " + self.config.device + " - trying to reconnect")
                await asyncio.sleep(1.0)
                try:
                    self.device.close()
                    self.device = evdev.InputDevice(self.config.device)
                    print("reconnected " + self.config.device)
                except:
                    print("reconnect failed")
            finally:
                if rospy.is_shutdown():
                        return

    async def run(self):
        await asyncio.wait({self.get_key_on_down_or_up()})


async def task_wrapper_ros_loop(pub: Publisher):
    await pub.prepare_pipeline()
    while not rospy.is_shutdown():
        await pub.update()
        # await pub.update()
        # r.sleep()
        await asyncio.sleep(0.01)
    return


async def call_wrapper(configs, pub: Publisher):
    tasks = set()
    tasks.add(asyncio.create_task(task_wrapper_ros_loop(pub)))
    for conf in configs:
        task = asyncio.create_task(ros_wrapper(conf, pub).run())
        tasks.add(task)
    await asyncio.wait(tasks)


if __name__ == '__main__':
    # store current tty settings
    tty_settings = termios.tcgetattr(sys.stdin)

    read_attempt_failed = False
    configs = []
    sys.stdout.write("Trying to read config ... ")
    file = "usb_keyboard.json"
    if len(sys.argv)>1 and len(sys.argv[1])>0:
        file = sys.argv[1]
    try:
        config_file = open(file, "r")
        try:
            config_json = json.load(config_file)
            for item in config_json:
                conf = usb_config()
                conf.__dict__ = item
                configs.append(conf)
        except:
            print("the usb_keyboard.json seems to be currupted and could not be parsed.")
            sys.exit(-1)
        finally:
            config_file.close()
    except:
        read_attempt_failed = True

    if read_attempt_failed:
        print("usb_keyboard.json not found or corrupted, only direct window input supported")
    else:
        sys.stdout.write("config found, starting node ")
        sys.stdout.flush()
        # asyncio.get_event_loop().run_until_complete(ros_wrapper(config).run())

    rospy.init_node('adore_supress_langechanges', anonymous=True)
    pub = Publisher()
    r = rospy.Rate(100)
    # set stdin raw mode to capture keystrokes
    tty.setcbreak(sys.stdin)
    print("Inputs:    W")
    print("         A S D")
    print("")
    print(" Press W to start langechange suppression - force lanefollowing")
    print(" Press A to force left  langechanges only")
    print(" Press D to force right langechanges only")
    print(" Press S to force slow maneuvers")
    print(" or ESC to exit node")

    # start all loops
    asyncio.get_event_loop().run_until_complete(call_wrapper(configs, pub))

    # restore tty settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, tty_settings)
