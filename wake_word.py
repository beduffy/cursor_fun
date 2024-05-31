import pvporcupine
import pyaudio
import struct
import os


class WakeWordListener:
    def __init__(self, access_key, keyword_path, callback):
        self.access_key = access_key
        self.keyword_path = keyword_path
        self.callback = callback
        self.porcupine = None
        self.audio_stream = None

    def initialize(self):
        self.porcupine = pvporcupine.create(
            access_key=self.access_key,
            keyword_paths=[self.keyword_path]
        )
        self.audio_stream = pyaudio.PyAudio().open(
            rate=self.porcupine.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.porcupine.frame_length
        )

    def listen(self):
        try:
            print("Listening for wake word...")
            while True:
                pcm = self.audio_stream.read(self.porcupine.frame_length)
                pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)

                keyword_index = self.porcupine.process(pcm)
                if keyword_index >= 0:
                    print("Wake word detected!")
                    self.callback()
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.cleanup()

    def cleanup(self):
        if self.porcupine is not None:
            self.porcupine.delete()
        if self.audio_stream is not None:
            self.audio_stream.close()
        pyaudio.PyAudio().terminate()

def wake_word_callback():
    print("Wake word callback triggered!")

if __name__ == "__main__":
    ACCESS_KEY = os.getenv("PORCUPINE_ACCESS_KEY", "default_access_key")
    KEYWORD_PATH = "Hello-John_en_linux_v3_0_0.ppn"

    listener = WakeWordListener(ACCESS_KEY, KEYWORD_PATH, wake_word_callback)
    listener.initialize()
    listener.listen()
