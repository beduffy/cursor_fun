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
        self.start_time = None
        self.video_fps = 30.0
        self.audio_rate = 44100


    def video_record(self):
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # or cv2.CAP_GSTREAMER
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(f'{self.filename}.avi', fourcc, self.video_fps, (640, 480))

        frame_duration = 1.0 / self.video_fps
        next_frame_time = self.start_time

        while self.is_recording:
            current_time = time.time()
            if current_time >= next_frame_time:
                ret, frame = cap.read()
                if ret:
                    out.write(frame)
                    cv2.imshow('Recording...', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                next_frame_time += frame_duration
            if current_time - self.start_time > self.duration:
                break

        cap.release()
        out.release()
        cv2.destroyAllWindows()


    def audio_record(self):
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 2

        p = pyaudio.PyAudio()
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=self.audio_rate,
                        input=True,
                        frames_per_buffer=CHUNK)

        time.sleep(0.1)  # Add a small delay before starting audio recording
        while self.is_recording:
            data = stream.read(CHUNK)
            self.audio_frames.append(data)
            if time.time() - self.start_time > self.duration:
                break

        stream.stop_stream()
        stream.close()
        p.terminate()

        wf = wave.open(f'{self.filename}.wav', 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(self.audio_rate)
        wf.writeframes(b''.join(self.audio_frames))
        wf.close()


    def start_recording(self):
        self.is_recording = True
        self.start_time = time.time()
        video_thread = threading.Thread(target=self.video_record)
        audio_thread = threading.Thread(target=self.audio_record)
        
        video_thread.start()
        audio_thread.start()
        
        video_thread.join()
        audio_thread.join()


from moviepy.editor import VideoFileClip, AudioFileClip, CompositeVideoClip

def combine_audio_video_moviepy(video_path, audio_path, output_path):
    # Load the video file
    video = VideoFileClip(video_path)
    
    # Load the audio file
    audio = AudioFileClip(audio_path)
    
    # Get the shorter duration
    min_duration = min(video.duration, audio.duration)
    
    # Trim both video and audio to the shorter duration
    video = video.subclip(0, min_duration)
    audio = audio.subclip(0, min_duration)
    
    # Ensure audio is at the same frame rate as the video
    audio = audio.set_fps(video.fps)
    
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
    combine_audio_video_moviepy("output.avi", "output.wav", "output_video.mp4")