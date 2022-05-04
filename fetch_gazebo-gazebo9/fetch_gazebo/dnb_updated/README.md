#DnB ROS Event:

##world instructions:

Extract the `models.zip` to a folder you desire. Note this should be in the `GAZEBO_MODEL_PATH` variable for gazebo to detect the models. To do that, from the directory you extracted the `models.zip` file, run 
```bash
GAZEBO_MODEL_PATH=$(pwd):$GAZEBO_MODEL_PATH
``` 
Then run the gazebo sim normally with the provided world.
