#!/usr/bin/python3

import pygame

import rospy
from std_msgs.msg import String

class SoundPlayer:

    def __init__(self):
        pygame.mixer.init()

        self.assets_folder = rospy.get_param("~assets_folder", None)
        self.priority_sounds = []

        if self.assets_folder is None:
            rospy.signal_shutdown("Need assets folder location")
            return

        self.sounds = {}
        self.addSound('fail', "fail.wav")
        self.addSound('success', "success.wav", True)
        self.addSound('out_of_time', "out_of_time.wav", True)

        self.command_sub = rospy.Subscriber("/play_sound/sound", String, self.handleSound)

    def addSound(self, trigger_str, file_name, has_priority=False):
        self.sounds[trigger_str] = pygame.mixer.Sound(self.assets_folder + file_name)
        if has_priority:
            self.priority_sounds.append(trigger_str)

    def handleSound(self, msg: String):
        is_busy = pygame.mixer.get_busy()
        if msg.data in self.sounds and not is_busy:
            self.sounds[msg.data].play()

        # If sound has priority, wait for channel to clear
        if is_busy and msg.data in self.priority_sounds:
            while is_busy:
                is_busy = pygame.mixer.get_busy()
        
            self.sounds[msg.data].play()

        

if __name__ == "__main__":
    rospy.init_node("sound_player", anonymous=True)
    player = SoundPlayer()
    rospy.spin()