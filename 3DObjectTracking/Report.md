# 3D Object Tracking with Lidar and Camera

## FP.1 Match Boxes
Implemented the method `matchBoundingBoxes`, which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches are the ones with the highest number of keypoint correspondences.

```
void matchBoundingBoxes(vector<cv::DMatch> &matches, map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
```

## FP.2 LIDAR TTC

Computed the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

Used median X-coordinates to remove the outliers.

```
double medPrev, medCurr;
int nPrev = validXPrev.size();
int nCurr = validXCurr.size();

medPrev = (nPrev % 2 == 0) ?
    (validXPrev[nPrev / 2 - 1] + validXPrev[nPrev / 2]) / 2.0 :
    validXPrev[nPrev / 2];

medCurr = (nCurr % 2 == 0) ?
    (validXCurr[nCurr / 2 - 1] + validXCurr[nCurr / 2]) / 2.0 :
    validXCurr[nCurr / 2];

if (medPrev <= medCurr) {
    TTC = NAN;
    return;
}
// compute TTC from both measurements
TTC = medCurr * dT / (medPrev - medCurr);
```

## FP.3 Camera Keypoints ROI
Prepared the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. 

Used the avg distance for the threshold.
```
double avgDist = 0.;
for (auto& match: kptMatches) {
    if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt)) {
        matchesROI.push_back(match);
        avgDist += match.distance;
    }
}

if (matchesROI.size() < 1) {
    return;
}
double threshold = 0.8 * avgDist / matchesROI.size();
for (auto match: matchesROI) {
    if (match.distance < threshold) {
        boundingBox.kptMatches.push_back(match);
    }
}
```

## FP.4 Camera TTC

Computed the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.


Used median distance with multiple checks involving the keypoint matches to avoid exploding values.


```
if (distRatios.size() == 0) 
{
    TTC = NAN;
    return;
}

sort(distRatios.begin(), distRatios.end());
int nDists = distRatios.size();
int medIdx = nDists / 2;
double medDistRatio = nDists % 2 == 0 ? (distRatios[medIdx] + distRatios[medIdx - 1]) / 2. : distRatios[medIdx];

if (medDistRatio == 1) 
{
    TTC = NAN;
    return;
}

double dT = 1 / frameRate;
TTC = -dT / (1 - medDistRatio);
```

## FP.5 Performance Evaluation 1

I was not able to find any frames where the lidar estimated TTC was unreasonable. It always ranges from about 8-16 seconds. I used the median approach over the minimum approach for a robust outlier-independent TTC calculation which works very well for this scenario. 

## FP.6

The `HARRIS` and `ORB` detectors provide unreliable TTC estimates for the camera-based TTC with many `NaN` values. This is likely due to insufficient matches and keypoints.

### Avg. Absolute TTC difference
|Detector\Descriptor|AKAZE|BRIEF|BRISK|FREAK|ORB|SIFT|
|---|---|---|---|---|---|---|
|AKAZE|1.3716|1.1257|0.9354|1.1125|1.1560|1.0170|
|BRISK|N/A|2.3714|3.0474|2.4118|2.1826|2.9566|
|FAST|N/A|1.5894|1.9780|1.8131|1.8117|1.6191|
|SHITOMASI|N/A|2.0172|1.8341|1.7765|1.9631|1.6689|
|SIFT|N/A|1.4418|1.2882|1.4600|N/A|1.2272|

### Avg Processing Time (ms) (post-YOLO)
|Detector\Descriptor|AKAZE|BRIEF|BRISK|FREAK|ORB|SIFT|
|---|---|---|---|---|---|---|
|AKAZE|183.71|88.70|385.6|119.83|93.01|176.58|
|BRISK|N/A|405.68|708.2|412.15|402.93|783.41|
|FAST|N/A|17.98|328.22|48.75|14.79|113.34|
|SHITOMASI|N/A|68.56|307.94|55.74|33.15|112.57|
|SIFT|N/A|107.89|412.7|134.26|N/A|262.35|

Based on the average TTC difference and processing times we can see that the three best combinations are:
1. `AKAZE/BRIEF`
2. `AKAZE/ORB`
3. `FAST/BRIEF`

These show reasonably close TTC estimates to the LIDAR-based TTC with very low runtimes.