import time
import json
import os
import subprocess
import pygame

PRESETS_FILE = 'presets.json'

class IntervalTrainingApp:
    def __init__(self):
        self.load_presets()
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption('Interval Training')
        self.font = pygame.font.Font(None, 74)
        self.clock = pygame.time.Clock()
        self.paused = False
        self.pause_start_time = None


    def load_presets(self):
        if os.path.exists(PRESETS_FILE):
            with open(PRESETS_FILE, 'r') as file:
                self.presets = json.load(file)
        else:
            self.presets = {}


    def save_presets(self):
        with open(PRESETS_FILE, 'w') as file:
            json.dump(self.presets, file, indent=4)


    def add_preset(self, name, events):
        self.presets[name] = events
        self.save_presets()


    def modify_preset(self, name, events):
        if name in self.presets:
            self.presets[name] = events
            self.save_presets()
        else:
            print(f"Preset '{name}' does not exist.")


    def calculate_total_duration(self, preset_name):
        if preset_name not in self.presets:
            print(f"Preset '{preset_name}' does not exist.")
            return 0

        events = self.presets[preset_name]
        total_duration = sum(event['duration'] for event in events)
        return total_duration


    def print_total_duration(self, preset_name):
        total_duration = self.calculate_total_duration(preset_name)
        hours, remainder = divmod(total_duration, 3600)
        mins, secs = divmod(remainder, 60)
        if hours > 0:
            print(f"Total workout duration: {total_duration} seconds ({hours} hr {mins} mins and {secs} seconds)")
        else:
            print(f"Total workout duration: {total_duration} seconds ({mins} minutes and {secs} seconds)")


    def start_training(self, preset_name):
        if preset_name not in self.presets:
            print(f"Preset '{preset_name}' does not exist.")
            return

        self.print_total_duration(preset_name)

        events = self.presets[preset_name]
        for event in events:
            next_event = self.get_next_event(events, event)
            print(f"Current: {event['name']} for {event['duration']} seconds. Next: {next_event}")
            self.speak(f"Starting {event['name']} for {event['duration']} seconds.")
            self.countdown(event['duration'], event['name'], next_event)
            if next_event != "Training complete!":
                self.speak(f"Next up: {next_event}")
                self.countdown(3, "Get Ready", next_event, pre_countdown=True)
            else:
                self.speak("Good job! Well done!")


    def countdown(self, duration, current_event, next_event, pre_countdown=False):
        start_time = time.time()
        while duration:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_p:
                        self.paused = not self.paused
                        if self.paused:
                            self.pause_start_time = time.time()
                        else:
                            paused_duration = time.time() - self.pause_start_time
                            start_time += paused_duration

            if self.paused:
                self.screen.fill((0, 0, 0))
                pause_text = self.font.render("Paused", True, (255, 255, 255))
                self.screen.blit(pause_text, (350, 250))
                pygame.display.flip()
                continue

            elapsed_time = time.time() - start_time
            remaining_time = duration - int(elapsed_time)
            if remaining_time <= 0:
                break

            mins, secs = divmod(remaining_time, 60)
            timeformat = '{:02d}:{:02d}'.format(mins, secs)
            self.screen.fill((0, 0, 0))
            time_text = self.font.render(timeformat, True, (255, 255, 255))
            current_event_text = self.font.render(f"Current: {current_event}", True, (255, 255, 255))
            next_event_text = self.font.render(f"Next: {next_event}", True, (255, 255, 255))
            self.screen.blit(time_text, (350, 250))
            self.screen.blit(current_event_text, (50, 50))
            self.screen.blit(next_event_text, (50, 150))
            pygame.display.flip()
            self.clock.tick(1)

        self.screen.fill((0, 0, 0))
        end_text = self.font.render("00:00", True, (255, 255, 255))
        self.screen.blit(end_text, (350, 250))
        pygame.display.flip()
        time.sleep(1)


    def get_next_event(self, events, current_event):
        current_index = events.index(current_event)
        if current_index + 1 < len(events):
            return events[current_index + 1]['name']
        else:
            return "Training complete!"


    def speak(self, text):
        subprocess.run(['espeak', text])

if __name__ == "__main__":
    app = IntervalTrainingApp()
    # app.add_preset("Sample Workout", [
    #     {"name": "Warm-up", "duration": 5},
    #     {"name": "Run", "duration": 120},
    #     {"name": "Walk", "duration": 60},
    #     {"name": "Cool-down", "duration": 60}
    # ])
    # app.add_preset("Sample Workout", [
    #     {"name": "Warm-up",   "duration": 5},
    #     {"name": "Run",       "duration": 120},
    #     {"name": "Walk",      "duration": 60},
    #     {"name": "Cool-down", "duration": 60}
    # ])
    # app.add_preset("Breaking 1hr 45min", [
    #     {"name": "Countdown",   "duration": 10},
    #     {"name": "Top rocks",       "duration": 5 * 60},
    #     {"name": "Shoulder",      "duration": 10 * 60},
    #     {"name": "Footwork",      "duration": 15 * 60},
    #     {"name": "Powermoves",      "duration": 25 * 60},
    #     {"name": "Freezes",      "duration": 15 * 60},
    #     {"name": "Choreo",      "duration": 5 * 60},
    #     {"name": "Combine/integrate/dance",      "duration": 5 * 60},
    #     {"name": "Back",      "duration": 10 * 60},
    #     {"name": "Stretch",      "duration": 15 * 60}
    # ])

    app.add_preset("Breaking 1hr 15min", [
        {"name": "Countdown",   "duration": 10},
        {"name": "Top rocks",       "duration": 5 * 60},
        {"name": "Shoulder",      "duration": 7 * 60},
        {"name": "Footwork",      "duration": 10 * 60},
        {"name": "Powermoves",      "duration": 25 * 60},
        {"name": "Freezes",      "duration": 10 * 60},
        {"name": "Combine/integrate/dance",      "duration": 5 * 60},
        {"name": "Back",      "duration": 5 * 60},
        {"name": "Stretch",      "duration": 8 * 60}
    ])

    app.add_preset("Breaking 1hr 30min", [
        {"name": "Countdown",   "duration": 10},
        {"name": "Top rocks",       "duration": 5 * 60},
        {"name": "Shoulder",      "duration": 7 * 60},
        {"name": "Footwork",      "duration": 5 * 60},
        {"name": "Powermoves",      "duration": 25 * 60},
        {"name": "Footwork 2",      "duration": 10 * 60},
        {"name": "Freezes",      "duration": 15 * 60},



        # TODO no back


        {"name": "Combine/integrate/dance",      "duration": 5 * 60},
        {"name": "Stretch",      "duration": 18 * 60}
    ])

    app.start_training("Breaking 1hr 15min")
    # app.start_training("Breaking 1hr 30min")
    # app.start_training("Breaking 1hr 45min")

    # TODO countdown starts after the next event is started or add these extra 3 second countdowns to full time?
    # TODO easy way to do minutes?
    # TODO dont say 300 seconds, always say minutes
    # TODO add functionality to pause and resume training. 
    # TODO add functionality to start training from specific place just in case i had to restart script
    # TODO add pygame screen with timer, current event, next event, etc.
    # TODO add ability to add presets from pygame
    # TODO one day could auto play music with toprocks, footwork and choreo etc

    # TODO able to talk to timer to ask how long is left, maybe easy with some wake word non api library

    # TODO talk to machine and tell it to shout ideas or combos at me
    # TODO tell me what to practice in footwork today etc
    # TODO youtube video about all of this
    # TODO human pose estimation and rebuild that startup that got 1B funding   