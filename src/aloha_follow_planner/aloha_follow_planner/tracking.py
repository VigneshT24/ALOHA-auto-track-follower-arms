"""Nominal joint-path timing and per-joint stall detection, independent of ROS."""
import bisect
import math


class JointPath:
    """Piecewise linear geometry with a joint-speed cap and no vertex skipping.

    Execution can pause this nominal clock. Paused wall time is never caught up
    by jumping ahead, and every original nonduplicate vertex remains in the path.
    Physical acceleration is handled by the existing servo profile.
    """
    def __init__(self, points, speed, period):
        if not points or not math.isfinite(speed) or speed <= 0 or not math.isfinite(period) or period <= 0:
            raise ValueError('Require a nonempty path and finite positive speed/period')
        self.points = []
        for point in points:
            if len(point) != 6 or not all(math.isfinite(q) for q in point):
                raise ValueError('Require finite six-joint path positions')
            point = list(point)
            if self.points and max(abs(a-b) for a, b in zip(point, self.points[-1])) < 1e-10:
                continue
            self.points.append(point)
        self.period = period
        self.times = [0.]
        for a, b in zip(self.points, self.points[1:]):
            self.times.append(self.times[-1]+max(period, max(abs(y-x) for x,y in zip(a,b))/speed))
        self.duration = self.times[-1]

    def sample(self, path_time):
        if path_time <= 0:
            return list(self.points[0]), 0
        if path_time >= self.duration:
            return list(self.points[-1]), len(self.points)-1
        end = bisect.bisect_left(self.times, path_time)
        start = end-1
        fraction = (path_time-self.times[start])/(self.times[end]-self.times[start])
        return [a+fraction*(b-a) for a,b in zip(self.points[start],self.points[end])], end

    def advance(self, path_time, elapsed):
        """Cap at one control tick and the next vertex, even after a long delay."""
        if elapsed < 0 or not math.isfinite(elapsed):
            raise ValueError('Invalid tracking clock interval')
        next_index = bisect.bisect_right(self.times, path_time+1e-10)
        if next_index >= len(self.times):
            return self.duration
        return min(self.duration, self.times[next_index], path_time+min(elapsed,self.period))


class StallMonitor:
    """A moving joint may lag, but persistent error without forward motion fails."""
    def __init__(self, timeout, minimum_progress, error_threshold):
        self.timeout = timeout
        self.minimum_progress = minimum_progress
        self.error_threshold = error_threshold
        self.pending = [None]*6

    def update(self, now, measured, target):
        stalled = []
        for index, (position, goal) in enumerate(zip(measured,target)):
            error = goal-position
            if abs(error) <= self.error_threshold:
                self.pending[index] = None
                continue
            direction = 1 if error > 0 else -1
            baseline = self.pending[index]
            if (baseline is None or direction != baseline[2] or
                    direction*(position-baseline[1]) >= self.minimum_progress):
                self.pending[index] = (now,position,direction)
            elif now-baseline[0] >= self.timeout:
                stalled.append(index)
        return stalled
