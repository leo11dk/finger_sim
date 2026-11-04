"""Native Control sliders and minimal keyboard actions; no text overlays."""
from queue import SimpleQueue, Empty


class ViewerControls:
    def __init__(self, experiment):
        self.experiment = experiment
        self.paused = False
        self.queue = SimpleQueue()

    def key_callback(self, key):
        self.queue.put(key)

    def process(self):
        self.experiment.sync_controls()
        while True:
            try:
                key = self.queue.get_nowait()
            except Empty:
                break
            if key == 268:  # Home: release and reset, retaining mechanical parameters
                self.experiment.reset()
            elif key == 269:  # End: restore all defaults
                self.experiment.reset(defaults=True)
            elif key == ord(' '):
                self.paused = not self.paused
