# Map Bridge Algorithms

## Key Components

The map bridge exists to compress ros maps from images/occupancy grids to line segments that represent walls that a SLAMing robot has detected. The algorithm has three key parts. They are:

1. the `HoughLinesP` function from the openCV python module `cv2`. This function is a probabalistic version of the Hough Line detection algorithm, which returns a list of detected line segments
2. [Tavares and Padilha's proposed new method for line segment merging](https://pdfs.semanticscholar.org/0bc5/340216126591c39eb0eddb50465f08b44fa1.pdf?_ga=2.43722469.1905612132.1588881299-1546646111.1588881299). We built our own implementaation of this algorithm - there are others that existed available on sites like StackOverflow, but they were obtuse and unreliable.
3. A homebrewed greedy line pairing algorithm that decides which line segments to merge.

### General Approach

The top-level structure of the map bridge is as follows

1. Get a map from the `/map` ros topic
2. get metadata like the origin, and turn the `OccupancyGrid` in a x by y numpy array representing the map as a greyscale image. `OccupancyGrid`s come as 1-d arrays. Furthermore, they come with the top of the map at the end of the array, meaning that after it has been converted from a 1-d to 2-d array, the 2-d array must be verically flipped (the top-level array must be reversed). During the conversion, any occupied cell is marked as a white pixel. all other pixels are black. This is because `HoughLinesP` uses white to detect lines.
3. if the image is small enough, dialate that image to produce better Hough results
4. Put the greyscale image through `HoughLinesP` and get a list of line segments.
5. reduce the number of line segments by merging similar line segments
6. transform all the line segment endpoints by the map origin metadata and the transform from odom to map, converting endpoints from using pixels as units to meters
7. put all the information into a JSON and send it to redis

### Merging Two Line Segments

Tavares and Padilha's algorithm is straighforward but required slight modification

1. start by finding a centroid, defined by the weighted average of the two line segment's endpoints, with the line segment's length as the weight
2. then, define the angle of the new line. This is also done with a weighted average of the two input lines angles. A modification had to be made here:
    1. Tavares and Padilha state that if the difference in angles of the two line segments is greater than ninety degrees, then change the angle of the second line by +/- pi depanding on whether the angle is positive or negative. However, sometimes this approach can actually incrrease the nominal difference by pi
    2. the modification made is to always subtract pi from the larger of the two angles, with always produces the desired result.
3. Transform the coordinates of the endpoints to a new frame, with the centroid found in step 1 as the origin and the x axis passing through the centroid at the angle found in step 2, however, only the x coordinates of the transofrmed points will matter, so the y coordinates are not calculated
4. the points that are furthest from the centroid in either direction (positive and negative) in the new frame are the distance from the centroid of the new endpoints. These can be easly found as the `min` and `max` of the group of transformed x coordinates
5. using the distances found in step 4, the centroid from step 1 and the angle from step 2, find the new endpoints
6. return the new line segment

### Line Consolodation

The greedy algorithm surrounding the Tavares and Padilha algorithm is not too hard to follow:

1. sort a list of all line segments in descending order of length. Bucket hash all line segments with their endpoints as the keys
2. iterate through the list of line segments. for each line, generate set of all points within the specified radius of the line. Find all lines that have endpoints in that set from the line hash. only save the nearby lines that have a similar enough angle.
3. for each nearby similar angle line segment that is found, continue looking for lines that are nearby to those segments that have a similar angle to the original line. put the set of lines into a queue to be merged.
4. for each set of lines to be merged in the queue, use Taveres and Padilha to merge them one by one on to a master line segment. once all lines have been merged, remove them all from the line list and hash. add the new line to the list and hash.
repeat until an iteration yeild no members in the merge queue
