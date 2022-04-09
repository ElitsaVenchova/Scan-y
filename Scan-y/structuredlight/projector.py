import subprocess # създаване на подпроцеси
import vlc # python vlc module
import cv2 as cv

"""
    Управлява проектора
"""
class Projector:

    media_player = None
    TMP_PATTERN_FILE_NAME = "/tmpPattern.jpg"
    CALIBRATION_DIR = "./Projector_Calib" # Директория съдържаща снимките за калибриране и файла с резултата
    CHESS_BOARD_PATTERN = "/chessboard.jpg" # Файл с резултата от калибрирането

    # Стартира HDMI порта, ако не е
    def __init__(self, cameraPi):
        # pass
        subprocess.call('vcgencmd display_power 1', shell=True)
        self.pCalibrationRes = cameraPi.readCalibrationResult(self.CALIBRATION_DIR)

    # стартира VLC и задава full screen
    def start(self):
        # pass
        self.media_player = vlc.MediaPlayer('--vout mmal_vout')
        self.media_player.toggle_fullscreen()

    # Задава изображение за показване на проектора
    def playImage(self, image):
        cv.imwrite(self.CALIBRATION_DIR + self.TMP_PATTERN_FILE_NAME, image)
        self.playImageByPath(self.CALIBRATION_DIR + self.TMP_PATTERN_FILE_NAME)

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
