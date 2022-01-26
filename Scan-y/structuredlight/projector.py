import subprocess # създаване на подпроцеси
import vlc # python vlc module
import cv2 as cv

"""
    Управлява проектора
"""
class Projector:

    media_player = None
    TMP_PATTERN_FULL_PATH = "/tmp/tmpPattern.jpg"
    CALIBRATION_DIR = "Projector_Calib" # Директория съдържаща снимките за калибриране и файла с резултата
    CALIBRATION_FILE = "./"+ CALIBRATION_DIR + "/ProjectorCalibrationResult.json" # Файл с резултата от калибрирането
    CHESS_BOARD_PATH = "./chessboard.png" # Файл с резултата от калибрирането

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
        cv.imwrite(self.TMP_PATTERN_FULL_PATH, image)
        self.playImageByPath(self.TMP_PATTERN_FULL_PATH)
        
    # Задава изображение по определ PATH за показване на проектора
    def playImageByPath(self, path):
        # pass
        media = vlc.Media(path)
        # setting media to the media player
        self.media_player.set_media(media)
        # start playing media
        self.media_player.play()

    # Спира VLC
    def stop(self):
        # pass
        self.media_player.toggle_fullscreen()
        self.media_player.stop()
