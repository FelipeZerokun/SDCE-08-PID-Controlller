## PID controller Writeup 

---

**PID Controller Project**
by Felipe Rojas

The goals / steps of this project are the following:

* Build a PID controller for a car in a track.
* Tune the PID hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!
* No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). 


## [Rubric](https://review.udacity.com/#!/rubrics/1972/view) Points

## Here I will consider the rubric points individually and describe how I addressed the problem

---

### PID Tuning

It was recommended to use the Twiddle algorithm to tune the parameters. I used manual tuning to test, one by one, how the parameters affected the car controller behaviour.

### My code

I began with the initialization. Initialized the Tao gains for P, I, D, and also the errors. I added a previous and sum of error for D and I. However, the sum of error variable was not needed.
* Code line from 11 to 33 in the PID.cpp
Then, the UpdateError method is for updating the errors for each PID parameters, depending on CTE;
* Code line from 35 to 48 in the PID.cpp
The last method I need is the TotalError, where I multiply the tao with the errors and add them up.
* Code line from 50 to 56 in the PID.cpp

With these methods, I just need to apply the method in the main.cpp
* Code line from 118 to 131.

Before, I wanted to apply a Twiddle method, but I stayed with manual tuning for now.
All my test goes from code line 41 to 78.

With the P controller I noticed the car would go faster to the middle of the lane (faster error reduction). However, this would make the car m oscillate a lot (overshooting). I lowered the P constant until I felt the car was not wiggling.
 I added the D value for a PD controller. the D value helped the car to get a faster to the center lane without much wiggle. At certain point, the PD controller was almost good enough. In the curves the car went too much to the side. After some tuning, I saw that this behaviour could not be improved, so I added the I value.
 Finally, tuning the PID with all the gains, I noticed the I parameter made the car over compensate so I had to lowered it a lot. After some tests, I found the gains values to make the car goes within the track without going out of the borders of the lane.

## Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

This was a simple project, where I was able to see how the PID controller works. It would have been faster and more accurate if I had implemented the Wiggle class within the project. However, due to my inexperience in C++, I decided to stick to the manual tuning. Like I said before, the tuning could have been better with more precise P, I, D parameters, but in the end was good enough to make the car drive itself in the track.