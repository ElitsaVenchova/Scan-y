import subprocess

# importing vlc module
import vlc
# importing time module
import time

try:
    # turn off hdmi
#     subprocess.call('vcgencmd display_power 0', shell=True)
#     time.sleep(5)
    # turn on hdmi
    subprocess.call('vcgencmd display_power 1', shell=True)

    # creating vlc media player object
    media_player = vlc.MediaPlayer()
    # toggle full screen
    media_player.toggle_fullscreen()
    # media object
    media = vlc.Media("image0.jpg")
    # seting media to the media player
    media_player.set_media(media)
    # start playing media
    media_player.play()
    time.sleep(15)
    print(1)
    # media object
    media = vlc.Media("image1.jpg")
    media_player.set_media(media)
    media_player.play()
    time.sleep(15)
    print(3)
    media_player.stop()
    print(2)
finally:
    # turn on hdmi
    subprocess.call('vcgencmd display_power 1', shell=True)
