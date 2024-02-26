#!/usr/bin/env python

"""
Simple example showing how to use the SoundClient provided by libsoundplay,
in blocking, non-blocking, and explicit usage.
"""

import rospy
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest


def play_explicit():
    rospy.loginfo('Example: SoundClient play methods can take in an explicit'
                  ' blocking parameter')
    soundhandle = SoundClient()  # blocking = False by default
    rospy.sleep(0.5)  # Ensure publisher connection is successful.

    sound_beep = soundhandle.waveSound("fword.wav", volume=0.3)
    # Play the same sound twice, once blocking and once not. The first call is
    # blocking (explicitly specified).
    sound_beep.play(blocking=True)
    # This call is not blocking (uses the SoundClient's setting).
    sound_beep.play()
    rospy.sleep(0.5)  # Let sound complete.
    rospy.sleep(1)


def play_blocking():
    """
    rospy.loginfo('Example: Playing sounds in *blocking* mode.')
    soundhandle = SoundClient(blocking=True)

    rospy.loginfo('Playing say-beep at volume 0.3.')
    soundhandle.playWave('fword.wav', volume=0.6)
    Play various sounds, blocking until each is completed before going to the
    next.
    """
    rospy.loginfo('Example: Playing sounds in *blocking* mode.')
    soundhandle = SoundClient(blocking=True)

    rospy.loginfo('Playing say-beep at volume 0.3.')
    soundhandle.playWave('fword.wav', volume=0.6)



def play_nonblocking():
    """
    Play the same sounds with manual pauses between them.
    """
    rospy.loginfo('Example: Playing sounds in *non-blocking* mode.')
    # NOTE: you must sleep at the beginning to let the SoundClient publisher
    # establish a connection to the soundplay_node.
    soundhandle = SoundClient(blocking=False)
    rospy.sleep(1)

    # In the non-blocking version you need to sleep between calls.
    rospy.loginfo('Playing say-beep at full volume.')
    soundhandle.playWave('say-beep.wav')
    rospy.sleep(1)

    rospy.loginfo('Playing say-beep at volume 0.3.')
    soundhandle.playWave('say-beep.wav', volume=0.3)
    rospy.sleep(1)

    rospy.loginfo('Playing sound for NEEDS_PLUGGING.')
    soundhandle.play(SoundRequest.NEEDS_PLUGGING)
    rospy.sleep(1)

    rospy.loginfo('Speaking some profanity')
    soundhandle.playWave('fword.wav', volume=1)
    # Note we will return before the string has finished playing.


if __name__ == '__main__':
    rospy.init_node('soundclient_example', anonymous=False)
    #play_explicit()
    play_blocking()
    #play_nonblocking()
    rospy.loginfo('Finished')