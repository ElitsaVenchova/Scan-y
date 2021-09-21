import numpy as np
from cv2 import VideoWriter, VideoWriter_fourcc

fname = '_sine_pattern_gen_'+str(60)+'_fps.avi'
video = VideoWriter(fname, VideoWriter_fourcc(*'MP42'), 60, (346, 260), isColor=False)

x = np.arange(346)  # generate 1-D sine wave of required period
y = np.sin(2 * np.pi * x / 20)

y += max(y) # offset sine wave by the max value to go out of negative range of sine

frame = np.array([[y[j]*127 for j in range(346)] for i in range(260)], dtype=np.uint8) # create 2-D array of sine-wave

for _ in range(0, 346):
    video.write(frame)
    shifted_frame =  np.roll(frame, 2, axis=1) # roll the columns of the sine wave to get moving effect
    frame = shifted_frame

video.release()