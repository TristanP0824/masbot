# Differential Drive Test Script

This script tests the differential drive controller by moving the robot in all four possible directions and collecting encoder readings for analysis.

## Test Sequence

The test script performs the following sequence:

1. **Forward Movement** (6 seconds)
2. **Backward Movement** (6 seconds)
3. **Left Turn** (3 seconds)
4. **Right Turn** (3 seconds)

Between each movement, there is a 1-second pause where the robot stops.

## Data Collection

During the test, the script collects the following data:
- Position (x, y)
- Linear velocity
- Angular velocity
- Movement type (FORWARD, BACKWARD, LEFT, RIGHT)
- Timestamp

All data is saved to a CSV file with a timestamp in the filename (e.g., `diff_drive_test_20250422_155823.csv`).

## Usage

Before running the test, make sure:
1. The robot is placed in an open area with enough space to move safely
2. The differential drive controller node is running

### Running the Test

To run the test:

```bash
# First, start the differential drive controller
ros2 launch /home/tristan/ros2_ws/src/diff_drive_controller_launch.py

# In a new terminal, run the test script
python3 /home/tristan/ros2_ws/src/diff_drive_test.py
```

### Test Results

After the test completes:
1. A CSV file with all collected data will be saved in the current directory
2. A summary of the test results will be printed to the console, including:
   - Distance traveled in each movement
   - Average linear velocity
   - Average angular velocity

## Analysis

The collected data can be used to:
- Verify the robot moves correctly in all directions
- Measure the accuracy of the odometry calculations
- Calibrate the wheel radius and track width parameters
- Analyze the performance of the motor controllers

You can use tools like Python with pandas/matplotlib or Excel to further analyze the CSV data.
