#!/bin/python3

import sys
import asyncio
import time
try:
    import rospy
    from std_msgs.msg import String
except:
    print("rospy not found, ROS nod sourced?")
    sys.exit()
import evdev
import json

# async def wait_helper(device):
#     return await get_key_on_down(device)

async def get_key_on_down(device):
    async for event in device.async_read_loop():
        if event.type == 1 and event.value == 1:
            return event

async def get_device_path_on_hold(device):
    async for event in device.async_read_loop():
        if event.value == 2:
            return device.path

class  usb_config:
    def __init__(self):
        self.device = ""
        self.keymap = {}
        self.topic = "keyboard"

### function to determine which usb device should be used for input and record all the keys to look for
async def find_device_holding_key():        
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    
    if len(devices) == 0:
        print("Found no USB devices. Check if current user is in the correct permission group to access /dev/input/, usually 'input'.")
        return
    futures = []
    device_found = ""
    
    for device in devices:
        futures.append(asyncio.ensure_future(get_device_path_on_hold(device)))
    
    for i in range(0,10):
        for future in futures:
            if future.done():
                device_found = future.result()
        if device_found != "":
            for future in futures:
                future.cancel()
            sys.stdout.write("\r\x1b[K\nGot response from: " + str(device_found) + "\n")
            break    
        sys.stdout.write("\r\x1b[KListening for keyboard events " + str(10-i))
        await asyncio.sleep(1)
    if device_found == "":
        print("None of the usb devices returned a key hold event, the device you want to use may not be compatible. But you could try again.")
        return # break here if no device could be determined
    
    sys.stdout.write("\r\x1b[K\nTo proceed press the first key you want to register within 10 seconds.\n")
    
    for i in range(0,3):
        sys.stdout.write("\r\x1b[Kstarting in " + str(3-i))
        time.sleep(1)
    sys.stdout.write("\r\x1b[K")
    
    key_codes = []
    device = evdev.InputDevice(device_found)
    task = asyncio.create_task(get_key_on_down(device))
    done, pending = await asyncio.wait({task},timeout=10)
    
    if task in done:
        key_code = evdev.categorize(task.result()).keycode
        print(key_code)
        key_codes.append(key_code)
    else:
        print("It seems like no key was pressed, aborting.")
        return # give up if no key is pressed
    print("First key registered, now keep pressing other keys to register their codes.\n" + \
          "To finish this process press a registered key again or wait for the 10 seconds after the last key (timeout)")
    keep_going = True
    while keep_going:
        task = asyncio.create_task(get_key_on_down(device))
        try:
            done, pending = await asyncio.wait({task},timeout=10)
        except:
            pass

        if task in done:
            key_code = evdev.categorize(task.result()).keycode
            
            if key_code in key_codes:
                keep_going = False
                break
            else:
                print(key_code)
                key_codes.append(key_code)
        else:
            task.cancel()
            keep_going = False

        await asyncio.sleep(0.1)

    if len(key_codes) > 0:
        print("finished registering keys")

    cfg = usb_config()

    cfg.device = device_found
    cfg.topic = "keyboard"
    
    key_count = 1
    for key in key_codes:
        if isinstance(key,list):
            key = key[0]
        cfg.keymap[key] = {"down" : str(key_count) + "_down", "up" : str(key_count) + "_up", "hold" : str(key_count) + "_hold"}
        key_count += 1

    try:
        config_file = open("usb_keyboard.json","w")
        json.dump([cfg.__dict__],config_file)
    except Exception:
        print("Could not write the config. Check permissions.")
    return 


class ros_wrapper:
    def __init__(self,config: usb_config):
        self.device = evdev.InputDevice(config.device)
        print(config.keymap)
        self.config = config
        rospy.init_node('adore_keyboard_input_node', anonymous=True)
        self.pub = rospy.Publisher(config.topic,  String,queue_size=10)

    async def write_msg(self,message):
        self.pub.publish(message)

    async def get_key_on_down_or_up(self):
        print("... listening for keypresses ")
        while not rospy.is_shutdown():
            async for event in self.device.async_read_loop():
                if event.type == 1:
                    print(event)
                    keycode = evdev.categorize(event).keycode
                    if isinstance(keycode,list):
                        keycode = keycode[0]
                    print(keycode)
                    if keycode in self.config.keymap:
                        if event.value == 0:
                            if "up" in self.config.keymap[keycode]:
                                print(keycode + " up")
                                await self.write_msg(self.config.keymap[keycode]["up"])
                        if event.value == 1:
                            if "down" in self.config.keymap[keycode]:
                                print(keycode + " down")
                                await self.write_msg(self.config.keymap[keycode]["down"])
                        if event.value == 2:
                            if "hold" in self.config.keymap[keycode]:
                                print(keycode + " hold")
                                await self.write_msg(self.config.keymap[keycode]["hold"])
                if rospy.is_shutdown():
                    return

    async def rospy_loop(self): # actual ros loop not needed, but this works to tear down the node when the roscore is torn down
        # r = rospy.Rate(10) 
        while not rospy.is_shutdown():
            # sys.stdout.write(".")
            # sys.stdout.flush()
            # self.pub.publish("test")
            await asyncio.sleep(0.1)
            # r.sleep()
        return

    async def run(self):

        # task = asyncio.create_task(self.get_key_on_down_or_up(config))
        # task2 = asyncio.create_task(self.get_key_on_down_or_up(config))
        
        # await asyncio.get_event_loop().run_in_executor(None,func=rospy_loop) 
        # await asyncio.wait(self.get_key_on_down_or_up(config),self.rospy_loop())
        await asyncio.wait({self.rospy_loop(),self.get_key_on_down_or_up()})

async def call_wrapper(configs):
    tasks = set()
    for conf in configs:
        task = asyncio.create_task(ros_wrapper(conf).run())
        tasks.add(task)
    await asyncio.wait(tasks)


if __name__ == "__main__":
    read_attempt_failed = False
    configs = []
    sys.stdout.write("Trying to read config ... ")
    file = "usb_keyboard.json"
    if len(sys.argv)>1 and len(sys.argv[1])>0:
        file = sys.argv[1]
    try:
        config_file = open(file,"r")
        try:
            config_json = json.load(config_file)
            for item in config_json:
                conf = usb_config()
                conf.__dict__ = item
                print(conf.keymap)
                configs.append(conf)
        except:
            print("the usb_keyboard.json seems to be currupted and could not be parsed, delete it and restart this node to create a new one.")
            sys.exit(-1)
        finally:
            config_file.close()
    except:
        read_attempt_failed = True
    
    if read_attempt_failed:
        print("usb_keyboard.json not found, creating new one. Hold down a key on the input device you want to configure")
        asyncio.get_event_loop().run_until_complete(find_device_holding_key())
    else:
        sys.stdout.write("config found, starting node ")
        sys.stdout.flush()
        asyncio.get_event_loop().run_until_complete(call_wrapper(configs))
        # asyncio.get_event_loop().run_until_complete(ros_wrapper(configs[0]).run())