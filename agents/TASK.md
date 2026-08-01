# Robot Trajectory Editor

## Brief

This is the source code of a web-based robot trajectory editor. It allows users to create, edit, and visualize robot trajectories in a user-friendly interface. The editor supports various robot models and provides tools joint position sequence editing, robot position sequence editiing, and trajectory playback.

## Features

### Loading and Saving Trajectories

- Load the existing robot trajectory file in `.npz` format.

- Save the current trajectory to a `.npz` file for future use.

- Each `.npz` file contains the following data:
  - `framerate`: a np array of Size[] type float32, representing the framerate of the trajectory.
  - `joint_names`: a 1D np array of type string, representing the names of the robot joints. It's ordered according to the joint position sequence.
  - `joint_pos`: a 2D np array of Size[frame_count, joint_count] type float32, representing the joint position sequence of the robot trajectory.
  - `base_pos_w`: a 2D np array of Size[frame_count, 3] type float32, representing the robot base position sequence in world coordinates (in x,y,z order)
  - `base_quat_w`: a 2D np array of Size[frame_count, 4] type float32, representing the robot base orientation sequence in world coordinates (in w,x,y,z order)

### User Interface

- The editor provides a main window with a 3D visualization of the robot and its trajectory, as well as the terrain mesh if loaded.

    - If no terrain mesh loaded, create a flat ground plane at z=0 for the robot to stand on.

- On the left, there is a panel for loading and saving trajectories, as well as selecting the robot model (if the launch CLI does not provide a robot urdf path)

    - A "Load Trajectory" button allows users to load an existing trajectory file in `.npz` format.

    - A "Load Terrain" button allows users to load a terrain file in `.obj` format. The terrain mesh file has the same coordinate system as the robot trajectory, and the robot will be placed on the terrain when the trajectory is played.

    - A "Save Trajectory" button allows users to save the current trajectory to a `.npz` file. (prompt the use to select a file path)

    - A "Edit Terrain" button allows the user to edit the terrain mesh in a pop-up window. The user can crop, down-sample, move, and rotate the terrain mesh. If the window is closed, the terrain mesh will be updated in the main window.

    - A "Save Terrain" button allows users to save the current terrain mesh to a `.obj` file.

    - A "Question Mark" button that opens a pop-up window with instructions on how to use the editor.

- On the left, there is a panel for selecting critical robot state target, such as joint name (for joint position), base position, and base orientation.

- On the bottom, there should be a drag-and-drop timeline for editing the joint position sequence and robot position sequence when selected. Users can add, remove, and modify keyframes in the timeline.

- Using "Shift" + mouse-drag to select a segment of the timeline. The selected segment will be highlighted in the timeline.

    - If a timeline segment is selected, "Up" or "Down" arrow key will move the value of the selected target up or down by a small step for all frames in the selected segment.

- On the right, there is a panel for displaying the current robot state, including joint positions, base position, and base orientation. Users can also manually input values to update the robot state.

- On the right, there is a panel for special operation buttons, such as "Fill", "Smooth", "Play", "Pause".

    - "Fill": Automatically fills the selected `Edit Target` channel with its value at the current frame when clicked. If a segment of the timeline is selected, only that segment of the channel will be filled. If no segment is selected, the entire channel will be filled. All other channels are left unchanged.

    - "Smooth": Removes high-frequency jitter from the selected state trajectory (low-pass filtering) when clicked, while preserving the overall shape of the motion. If a segment of the tlimeline is selected, only that segment will be smoothed. If no segment is selected, the entire trajectory will be smoothed.

    - "Interpolate": Interpolate the selected segment of the trajectory using linear interpolation from the start frame to the end frame. If no segment is selected, no interpolation will be performed.

- In the main 3D window, where the user can click and drag the robot to change its current position and orientation.

    - All cooresponding state (either joint position or base position/orientation) will be updated in the timeline and the robot state panel.

    - If the robot double-clicked any body link, the editor should make that link fixed in the world coordinate system, and the user can drag the robot to change its base position and orientation. The joint positions will be updated accordingly.

        - An alert message should be displayed on the top of the main window to inform the user that the link is fixed in the world coordinate system.

        - The mesh of the fixed link should be highlighted in the main 3D window.

        - The user can double-click the fixed link again to unfix it, and the editing mode will return to the default mode where the robot body can be dragged to change its joint positions.

        - Allowing multiple links to be fixed in the world coordinate system is required functionality. The user can double-click any link to fix it, and double-click the same link again to unfix it.

- When the `Edit Target` is selected. The user pressing "Up" or "Down" arrow key shall move the value of the selected target up or down by a small step. The step size can be adjusted in the launch terminal.

- No matter whether the `Edit Target` is selected. As long as the motion is loaded, the user pressing "Left" or "Right" arrow key shall move the value of the selected target left or right by a small step. The step size can be adjusted in the launch terminal.

- When the user press "Space" key, the trajectory playback will be toggled between play and pause.

### Implementation Requirements

- The robot state must be stored as a dedicated class, which contains the joint position sequence, base position sequence, and base orientation sequence. The class should provide methods for loading and saving the state to a `.npz` file, as well as methods for updating the state based on user input.

- The editor should be implemented using a web framework (e.g., React, Vue.js) for the frontend and a backend server (e.g., Flask, Django) to handle file loading and saving.

- Please use common libraries for 3D visualization (e.g., Three.js, Babylon.js) to render the robot and terrain in the main window.

- If the CLI launch command provides a robot urdf path, the urdf path could be as follows:

```
python3 -m agents.robot_trajectory_editor --robot_urdf_path /path/to/robot.urdf
```

which means the path is a local file path, or 

```
python3 -m agents.robot_trajectory_editor --robot_urdf_path package://robot_description/urdf/robot.urdf
``` 

which means the path is a python package path, where `robot_description` is the name of the package, and `urdf/robot.urdf` is the relative path to the urdf file in the package.
