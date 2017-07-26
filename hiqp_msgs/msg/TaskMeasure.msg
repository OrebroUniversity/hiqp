# The HiQP Control Framework, an optimal control framework targeted at robotics
# Copyright (C) 2016 Marcus A Johansson
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

string         task_name     # unique string identifier for the task
int32          EQ = 0        # Equality task
int32          GEQ = 1       # Greater than or equal to
int32          LEQ = -1      # Less than or equal to
int32          task_type     # Task Type (eq, geq or leq as above)
float64[]      e             # the task function value at this time step (task space control error)
float64[]      de            # the task function derivative at this time step (task space error derivative)
float64[]      dde_star      # desired task function acceleration (desired task space error acceleration according to the control law)
float64[]      pm            # the user-defined custom performance monitoring values for this task at this time step
