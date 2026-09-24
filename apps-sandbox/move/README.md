# Drive a square

Drives a 200 mm square on the per-wheel speed loop, then blinks. Each side and
each corner is an odometric goal: the wheels run until the encoders say the
distance is done, then brake to a stand. No position fix is used. A stalled
wheel (held at high duty without turning, against an obstacle) ends the square
there.
