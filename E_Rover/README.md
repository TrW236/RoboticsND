## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[warped]: img/warped.png
[threshed]: img/threshed.png
[image3]: calibration_images/example_rock1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

I used thresholding to distinguish obstacles and roads out of the received images, due to that the roads are bright, but the rocks are dark. The function to perform this functionality named as `color_thresh`. To find the rocks, thresholding is also applied, because of the yellow color of the rocks, see the function `find_rocks`.

![warped][warped]
![threshed][threshed]

#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

```python
    data.worldmap[y_world, x_world, 2] = 255
    data.worldmap[obs_y_world, obs_x_world, 0] = 255
    nav_pix = data.worldmap[:, :, 2] > 0
    data.worldmap[nav_pix, 0] = 0
```

The above codes are to generate the processed world map. The `data.worldmap` contains already the ground truth world map with green on representing the rodes and black representing the rocks.
First, the pixels for detected roads are set as blue.
Then, the pixels for obstacles are set as red. The output video is in the `output` folder.

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

The raw image received from the simulator is at first warped as the bird-view using the function `perspect_transform`. Then, the obstacles and the driveable roads are separated through thresholding `color_thresh`.  The pixel indexes are mapped to the world using `pix_to_world`. In the end, the world image, which is saved in the rover, is modified according to the world pixel indexes of obstacles, rocks, and roads.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

I used the `win 64` simulator, and the resolution was `720x576`, quality `Fantastic`. The result was good. The rover could be able to detect the rock and pick the rock up. The main reason why these approaches performed well in the simulator is that the images to be processed are captured from the simulator, where the rocks, obstacles and driveable roads are easy to detect due to the different colors they have. If I were to pursue this project, I would use machine learning techniques to identify the rocks and roads more generally. (In reality, the rover will use many other sensors working together to perform this sort of tasks.)

![](img/res.jpg)