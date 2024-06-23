import cv2
import pyaudio
import wave
import numpy as np
import threading
import time

class VideoAudioRecorder:
    def __init__(self, filename, duration):
        self.filename = filename
        self.duration = duration
        self.frames = []
        self.audio_frames = []
        self.is_recording = False

    def video_record(self):
        cap = cv2.VideoCapture(0)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(f'{self.filename}.avi', fourcc, 20.0, (640, 480))

        start_time = time.time()
        while self.is_recording:
            ret, frame = cap.read()
            if ret:
                out.write(frame)
                cv2.imshow('Recording...', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            if time.time() - start_time > self.duration:
                break

        cap.release()
        out.release()
        cv2.destroyAllWindows()

    def audio_record(self):
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 2
        RATE = 44100

        p = pyaudio.PyAudio()
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)

        start_time = time.time()
        while self.is_recording:
            data = stream.read(CHUNK)
            self.audio_frames.append(data)
            if time.time() - start_time > self.duration:
                break

        stream.stop_stream()
        stream.close()
        p.terminate()

        wf = wave.open(f'{self.filename}.wav', 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(self.audio_frames))
        wf.close()

    def start_recording(self):
        self.is_recording = True
        video_thread = threading.Thread(target=self.video_record)
        audio_thread = threading.Thread(target=self.audio_record)
        
        video_thread.start()
        audio_thread.start()
        
        video_thread.join()
        audio_thread.join()

from moviepy.editor import VideoFileClip, AudioFileClip, CompositeVideoClip

def combine_audio_video(video_path, audio_path, output_path):
    # Load the video file
    video = VideoFileClip(video_path)
    
    # Load the audio file
    audio = AudioFileClip(audio_path)
    
    # Set the audio of the video clip
    final_clip = video.set_audio(audio)
    
    # Write the result to a file
    final_clip.write_videofile(output_path, codec='libx264', audio_codec='aac')
    
    # Close the clips
    video.close()
    audio.close()
    final_clip.close()

if __name__ == "__main__":
    recorder = VideoAudioRecorder("output", duration=10)  # Record for 10 seconds
    recorder.start_recording()


    # Usage
    combine_audio_video("output.avi", "output.wav", "output_video.mp4")