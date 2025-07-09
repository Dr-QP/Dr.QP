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

from typing import Callable

import rclpy.node


class TimedQueue:
    """A queue that allows to execute a sequence of actions with a delay between them."""

    def __init__(self, node: rclpy.node.Node):
        self.node = node
        self.queue = []
        self.timer = None

    def add(self, delay: float, action: Callable):
        self.queue.append((delay, action))
        self.__next()

    def __next(self):
        if not self.queue or self.timer:
            return

        delay, action = self.queue.pop(0)
        self.timer = self.node.create_timer(delay, self.execute)
        action()

    def execute(self):
        if self.timer:
            self.timer.destroy()
            self.timer = None

        self.__next()
