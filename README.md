# The HiQP Control Framework
Copyright (C) 2016 Marcus A Johansson

HiQP is an optimal control framework targeted at robotics.

# Known issues
Geometric projection for point on box is not working when the box is rotated.

Geometric alignment for line with cylinder is not working, probably the jacobian is miscalculated.

Adding a projection/alignemnt task that uses nonexisting geometric primitives results in segfault, should result in warning and no task added.
