# Copyright (c) 2017-2025 Anton Matosov
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from abc import abstractmethod

import matplotlib.pyplot as plt
import numpy as np

from point import Point3D, Point


class GaitGenerator:
    @abstractmethod
    def get_offsets_at_phase(self, phase, direction=Point(1, 0)) -> dict[str, Point3D]:
        pass

    @abstractmethod
    def get_offsets_at_phase_for_leg(self, leg, phase, direction=Point(1, 0)) -> Point3D:
        pass

    def visualize_continuous(self, steps=100, direction=Point(1, 0)):
        """Visualize the gait sequence as a continuous function."""
        phases = np.linspace(0, 1, steps, endpoint=True)

        # Create data structures to store values for plotting
        x_values = {leg: [] for leg in self.tripod_a + self.tripod_b}
        y_values = {leg: [] for leg in self.tripod_a + self.tripod_b}
        z_values = {leg: [] for leg in self.tripod_a + self.tripod_b}

        # Generate data points
        for phase in phases:
            offsets = self.get_offsets_at_phase(phase, direction)
            for leg, offset in offsets.items():
                x_values[leg].append(offset.x)
                y_values[leg].append(offset.y)
                z_values[leg].append(offset.z)

        # Plot the data
        fig, axs = plt.subplots(3, 1, figsize=(12, 10))

        # Plot x offsets
        for leg in self.tripod_a + self.tripod_b:
            if leg in self.tripod_a:
                axs[0].plot(phases, x_values[leg], label=f'{leg} (A)', linestyle='-')
            else:
                axs[0].plot(phases, x_values[leg], label=f'{leg} (B)', linestyle='--')
        axs[0].set_title('X offsets (forward/backward)')
        axs[0].set_ylabel('meters')
        axs[0].set_xlabel('phase')
        axs[0].legend()

        # Plot y offsets
        for leg in self.tripod_a + self.tripod_b:
            if leg in self.tripod_a:
                axs[1].plot(phases, y_values[leg], label=f'{leg} (A)', linestyle='-')
            else:
                axs[1].plot(phases, y_values[leg], label=f'{leg} (B)', linestyle='--')
        axs[1].set_title('Y offsets (left/right)')
        axs[1].set_ylabel('meters')
        axs[1].set_xlabel('phase')
        axs[1].legend()

        # Plot z offsets
        for leg in self.tripod_a + self.tripod_b:
            if leg in self.tripod_a:
                axs[2].plot(phases, z_values[leg], label=f'{leg} (A)', linestyle='-')
            else:
                axs[2].plot(phases, z_values[leg], label=f'{leg} (B)', linestyle='--')
        axs[2].set_title('Z offsets (up/down)')
        axs[2].set_ylabel('meters')
        axs[2].set_xlabel('phase')
        axs[2].legend()

        plt.tight_layout()
        plt.show()
