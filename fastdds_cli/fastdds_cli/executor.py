#!/usr/bin/env python3

import os
import signal
import subprocess
import sys

class Executor:

    def __init__(self, command_arg):
        self.command_arg = command_arg

    def execute(self):

        def signal_handler(sig, frame):
            print("\nSignal captured, ending node execution...")

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        try:

            result = subprocess.run(
                    ["fastdds", self.command_arg] + sys.argv[1:],
                    env=os.environ
                )

        except Exception as e:
            print('Error:', e)
            return
