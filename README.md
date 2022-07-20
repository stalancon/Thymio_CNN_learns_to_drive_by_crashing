# Learning-to-Drive-by-Crashing

## Description
Final project for robotics class where we were asked to do the following:

- Build a world in Gazebo with obstacles in a certain location.
- Run the MyT in the world manually such that it steers away from obstacles while collecting frames.
- Train a CNN on the dataset (collected frames)
- Run the MyT one more time in our world, but the obstacles are in a different position.


### Refer to the launch files in `launch` folder to:

1. Bring up gazebo and the MyT's model. You must name your simulated MyT with the `name` parameter, e.g. `thymio10`. In addition you must indicate `world` to simulate the original world:

```
roslaunch group_s thymio_gazebo_bringup.launch name:=thymio10 world:=world
```


### Refer to the obstacle_spawner node to:

Populate the world with the obstacles.

Choose which obstacles to use from the models that we have in the folder `models`.


```
rosrun group_s obstacle_spawner.py
```



### Refer to the training_wheels node to:

Make the thymio collect frames while randomly exploring the world. The frames are stored in the folder `frames` in the class that they were classified as.

```
rosrun group_s training_wheels.py
```

### Refer to the cnn node to:

This node was added to show the code we did for our CNN model, however we used the tool Google colab to run it. Our model `cnn_model.h5`, is stored in the following link: https://usi365-my.sharepoint.com/:u:/g/personal/talans_usi_ch/EczV39OILcBLtD2Gzq7CnvYByzGIFoiiZOiyFG_bWhwMZg?e=zcGapr , it can be put into the folder 'group_s' to be used in the node `explorer.py`.



### Refer to the explorer node to:

Test the thymio in the chosen world! 
Original world:  `world`
World with different textures for both floor and walls:  `world_1`
World with different texture for the floor:  `world_2`

The thymio explores the world by obtaining the predictions from our pre-trained cnn model.

```
rosrun group_s explorer.py
```
