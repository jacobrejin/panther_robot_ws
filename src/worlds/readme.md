# DARTeC World Files

### This workspcae contains the world files used for the simulation. These world files are created by editing the world files provided by Mr. Angelos. Which is the scaled model of the DARTeC enviornemnt.


<br>
<hr>


# There are 3 Different world files
<br>

## 1. dartec_wrold_1 (Default world which is the scaled replica of the DARTeC enviornment)

<div style="text-align: center;">
    <img src="images_for_readme/default_world.png" alt="Panther LIDAR" width="450">
</div>



<br>
<br>
<br>
<hr>



## 2. dartec_wrold_2 (Default world with waypoints added to the inspection points)

<div style="text-align: center;">
    <img src="images_for_readme/world_w_wp.png" alt="Panther LIDAR" width="450">
</div>



<br>
<br>
<br>
<hr>



## 3. dartec_wrold_2 (Default world with waypoints and the static obstacles added to random locations in the world)

<div style="text-align: center;">
    <img src="images_for_readme/world_w_wp.png" alt="Panther LIDAR" width="450">
</div>





<br>
<br>
<br>
<hr>



## 4. dartec_wrold_4 (This is a copy of the dartec_world_1 with added UWB Beacons) 

*But with some major chnages which inlcudes separing out the components of the hangar*

In the original world file, the hangar along with the other components like the plane were part of a single model file, this make them harder to handle as we want then to be seaprate models with each having their own collision for the UWB plugni to work (ray tracing). So I deciede to separate them into individual models and then add them to the world file. (only downsied being the world file is really hard to read)

Also a total of 9 UWB tags were added to the world file with the position approximated to the real world location of the UWB tags in the DARTeC enviornment.

<div style="text-align: center;">
    <img src="images_for_readme/default_world.png" alt="Panther LIDAR" width="450">
</div>


<!-- add the image of the collision for the previou and the current world -->