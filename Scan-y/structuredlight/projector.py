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
        subprocess.call('vcgencmd display_power 1', shell=True)
    
    # стартира VLC и задава full screen
    def start(self):
        self.media_player = vlc.MediaPlayer()
        self.media_player.toggle_fullscreen()
        
    # Задава изображение за показване на проектора
    def playImage(self, image):
        cv.imwrite(self.TMP_PATTERN_FULL_PATH, image)
        media = vlc.Media(self.TMP_PATTERN_FULL_PATH)
        # setting media to the media player
        self.media_player.set_media(media)
        # start playing media
        self.media_player.play()
        
    # Спира VLC
    def stop(self):
        self.media_player.stop()