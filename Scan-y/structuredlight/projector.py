import subprocess # създаване на подпроцеси
import vlc # python vlc module
import cv2 as cv

"""
    Управлява проектора
"""
class Projector:

    media_player = None
    TMP_PATTERN_FULL_PATH = "/tmp/tmpPattern.jpg"

    # Стартира HDMI порта, ако не е
    def __init__(self):
        # pass
        subprocess.call('vcgencmd display_power 1', shell=True)

    # стартира VLC и задава full screen
    def start(self):
        # pass
        self.media_player = vlc.MediaPlayer()
        self.media_player.toggle_fullscreen()

    # Задава изображение за показване на проектора
    def playImage(self, image):
        # pass
        cv.imwrite(self.TMP_PATTERN_FULL_PATH, image)
        media = vlc.Media(self.TMP_PATTERN_FULL_PATH)
        # setting media to the media player
        self.media_player.set_media(media)
        # start playing media
        self.media_player.play()

    # Спира VLC
    def stop(self):
        # pass
        self.media_player.stop()
