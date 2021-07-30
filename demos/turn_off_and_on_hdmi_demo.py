import subprocess

# importing vlc module
import vlc
# importing time module
import time

try:
    # turn off hdmi
    subprocess.call('vcgencmd display_power 0', shell=True)
    time.sleep(5)
    # turn on hdmi
    subprocess.call('vcgencmd display_power 1', shell=True)

    # creating vlc media player object
    media_player = vlc.MediaPlayer()
    # toggle full screen
    media_player.toggle_fullscreen()
    # media object
    media = vlc.Media("pexels-cup-of-couple-8014403.mp4")
    # setting start and stop time
    media.add_option("start-time=0.0")
    media.add_option("end-time=13.0")
    # seting media to the media player
    media_player.set_media(media)
    # start playing media
    media_player.play()
    time.sleep(15)
    media_player.stop()
finally:
    # turn on hdmi
    subprocess.call('vcgencmd display_power 1', shell=True)
