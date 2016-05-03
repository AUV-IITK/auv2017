# Build Instruction
```sh
catkin_make clean
catkin_make --pkg motion_commons
catkin_make
```

# Structure
The stack contains
  - `motion_commons` contains action files.
  - `motion_forward` forward motion
  - `motion_sideward` sway motion
  - `motion_turn` rotating in horizontal plain
  - `motion_upward` vertical motion
