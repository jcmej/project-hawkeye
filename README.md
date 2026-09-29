# project-hawkeye
ShellsHack 2026

## Inspiration
Car's like those that are apart of Waymo's fleets, are as cool to look at as they are expensive. With all their components like lidar, radar, and all the camera's within them, their creation, maintenance, and operation is a financial drain. But what if it didn't need to be. What if there was a way to make autonomous vehicles cheaper to develop and maintain, while keeping (if not improving) their navigation. These are some of the things we aimed to accomplish for this project.

## What it does
We've developed a end-to-end pipeline that takes real-time feed from camera/s (in our case, from iPhone's) from a developed and locally deployed app on our phones, to another local desktop app, that processes the information into navigational commands, and then sends them to our custom-built car. It is an autonomous process that gets us from point A to point B in the best way possible.

## How we built it
We started with 2 local apps, the 1st built for iPhone's using swift, to take in real-time live feed and turn it into coordinates that can be interpreted on a 2D plain. The 2nd we built using X-code for pc's, that processes the coordinates from the first app, and then turns them into commands for the car to follow. Then we had to change where the inputs came from. We needed to have command inputs be given from a pc, instead of the remote control that belonged to the car. So, we flashed the firmware, added our own logic (including some commands for changing the colors on the RGB lights), and rerouted where the car was listening for input, to our pc's via the car's own Wi-Fi signal. Throughout this process we also used ArUco markers, 4 for each corner of the environment we want to navigate, 1 for the goal destination, and 1 for the orientation of the car itself. Thus we were able to create the end-to-end pipeline that allows for autonomous interpretation and navigation.

## Interested? 
Why not watch how it works?
https://youtube.com/shorts/VWHUr4YO8QQ?feature=share

