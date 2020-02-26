### CameraMerging
| Step      | Description                | 
| ------------- |:------------------:|
|  1     | Video Capture (create dataset for calibration)    |
| 2  | Undistort images         |
|  3  | Images calibration using one of the given patterns( cheesboard, circles) |
|  4  | Calibaration of StereoPair |
|  5  | Calculate Field of View to check the difference between cameras frame) |
| 6  | Align Visible or Infrared frame to the desirable size |
| 7  | Using method of endless homography, for alignment improvement|
|  8  | Merge images using Canny |
|  9  | Merge images using Weights |
|  10 | Merge images using [GFF](https://github.com/L0rd1k/GFF-Cameras-frame-fusion) |


| Visible       | Infrared           |
| ------------- |:------------------:|
|![alt-текст](https://github.com/L0rd1k/CameraMerging/blob/master/TestResults/1.png)|![alt-текст](https://github.com/L0rd1k/CameraMerging/blob/master/TestResults/1-1.png)|
|![alt-текст](https://github.com/L0rd1k/CameraMerging/blob/master/TestResults/2.png)|![alt-текст](https://github.com/L0rd1k/CameraMerging/blob/master/TestResults/1-2.png)|
