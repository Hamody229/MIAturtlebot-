# Shape and Color Detection with OpenCV

## Overview

In this project, we focused on detecting basic geometric shapes such as triangles, rectangles, squares, and circles, as well as identifying their colors, including red, green, blue, and yellow. The task was approached using OpenCV, and only classical computer vision techniques were applied, without the use of deep learning models.

## Approach

### 1. Shape Detection
We employed the following steps to detect and classify shapes in the image:

- **Edge Detection**: Initially, the image was converted to grayscale, and Canny edge detection was applied to identify the edges within the image.
- **Contour Detection**: Contours were then extracted from the edges, representing the boundaries of the shapes.
- **Shape Classification**: To classify the shapes, we applied the `cv2.approxPolyDP()` function to approximate the contours as polygons. Based on the number of vertices, we classified the shapes as follows:
  - 3 vertices: Triangle
  - 4 vertices: Rectangle or Square (determined by the aspect ratio)
  - More than 4 vertices: Circle
- **Aspect Ratio**: For shapes with four vertices, the aspect ratio (width divided by height) was calculated. If the ratio was approximately 1, the shape was classified as a square; otherwise, it was identified as a rectangle.

### 2. Color Detection
To identify the colors of the detected shapes, we utilized the HSV color space, which is more suitable for color detection than RGB. The process involved the following steps:

- **Mask Creation**: A mask was created for each detected shape to isolate its region in the image.
- **Mean Color Calculation**: The mean color of the masked region was calculated to represent the dominant color of the shape.
- **HSV Conversion**: The mean color, initially in BGR format, was converted to HSV.
- **Color Matching**: The HSV value of the mean color was compared against predefined ranges for red, green, blue, and yellow to determine the shape's color.

### 3. Drawing and Labeling
Once the shapes and their corresponding colors were detected, we drew the contours of the shapes on the image. Labels indicating both the shape and its color (e.g., "Triangle (Red)") were added to the image.

## Algorithms and Techniques Used

### 1. Canny Edge Detection
Canny edge detection was employed to detect sharp changes in intensity within the image, which correspond to the edges of shapes. This method proved effective in identifying the boundaries of various shapes.

### 2. Contour Approximation
We used the `cv2.approxPolyDP()` function to approximate the contours of the shapes. This simplification process helped in classifying the shapes based on their vertices.

### 3. Aspect Ratio Calculation
For quadrilateral shapes (with four vertices), the aspect ratio was calculated. A near 1:1 ratio indicated a square, while other ratios pointed to a rectangle.

### 4. HSV Color Space
The HSV color space was chosen for color detection due to its separation of color information (hue) from intensity (value). We defined specific ranges for red, green, blue, and yellow in HSV format, which allowed for accurate color classification.

## Challenges and Insights

### Challenges
1. **Shape Detection**: A significant challenge was accurately detecting shapes, particularly inclined rectangles, which were sometimes misclassified. By refining the contour approximation and adjusting the aspect ratio checks, we improved the accuracy of shape detection.
   
2. **Color Detection**: Detecting colors was sometimes problematic, but we solved that by calculating the mean color within the masked regions, resulting in more consistent color identification.

### Insights
- This task demonstrated the effectiveness of classical computer vision techniques in solving basic shape and color detection problems. While modern deep learning methods may simplify such tasks, traditional methods like edge detection and contour approximation remain powerful tools for image processing.
- The HSV color space proved especially useful in color detection, as it allowed for more reliable separation of color information compared to the RGB model.
