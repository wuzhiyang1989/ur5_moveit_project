#!/home/wy/anaconda3/envs/daily/bin/python3
import os
import time
import rospy
import pygame 
from mutagen.mp3 import MP3
from hm_cooperation.srv import Mp3

def which_voice(cmd):
    if cmd[0] >11:
        print("To order cmd = ", [cmd[0]-11, cmd[1]-17, cmd[2]])
    else:
        print("To sort cmd  = ", cmd)
    if cmd == [-1, -1, -1]:
        return -1 
    if cmd[0] < 12:
        # print("Robot cmd: from %d to sort." % cmd[0])
        return cmd[0]
    if cmd[0] < 18 and cmd[0] > 11 and cmd[1] != 23:
        # print("Robot cmd: from sort %d to order %d." % (cmd[0]-12, cmd[1]-18))
        return (cmd[0]-12)*3+(cmd[1]-17)+11
    if cmd[1] == 23:
        # print("Robot cmd: from %d to trash." % (cmd[0]-12))
        return cmd[0]-12 + 30
    else :
        # print("="*80)
        return -1


def mp3_service(req):
    index = which_voice(req.cmd)
    if index == -1:
        return False
    else:
        mp3_file = "/home/wy/script/speech-demo/rest-api-tts/python/voice/voice_" + str(index) + ".mp3"
        length = MP3(mp3_file).info.length
        if os.path.exists(mp3_file):
            pygame.mixer.init()
            pygame.mixer.music.load(mp3_file)
            pygame.mixer.music.play()
            time.sleep(int(length+1))
        return True


if __name__ == '__main__':
    rospy.init_node("Mp3_service")
    s = rospy.Service('Mp3_player', Mp3, mp3_service)
    rospy.spin()