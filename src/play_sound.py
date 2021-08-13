#!/usr/bin/python3

import pygame

import rospy
from std_msgs.msg import String

class SoundPlayer:

    def __init__(self):
        pygame.mixer.init()

        self.assets_folder = rospy.get_param("~assets_folder", None)
        self.command_sub = rospy.Subscriber("/play_sound/sound", String, self.handleSound)

        if self.assets_folder is None:
            rospy.signal_shutdown("Need assets folder location")
            return

        self.sounds = {}
        self.addSound('fail', "fail.wav")
        self.addSound('success', "success.wav")


    def addSound(self, trigger_str, file_name):
        self.sounds[trigger_str] = pygame.mixer.Sound(self.assets_folder + file_name)    

    def handleSound(self, msg: String):
        if msg.data in self.sounds:
            self.sounds[msg.data].play()


if __name__ == "__main__":
    rospy.init_node("sound_player", anonymous=True)
    player = SoundPlayer()
    rospy.spin()