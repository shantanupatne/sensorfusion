
# Report

## MP.1:
Ring buffer using STL `deque` with `dataBufferSize = 2`.

```
deque<DataFrame> dataBuffer;
...
dataBuffer.push_back(frame);
if (dataBuffer.size() > dataBufferSize)
    dataBuffer.pop_front();
```

---

## MP.2:
Implemented `HARRIS`, `FAST`, `BRISK`, `ORB`, `SIFT`, `AKAZE` selectable by string.

```
if (detectorType.compare("SHITOMASI") == 0)
{
    detKeypointsShiTomasi(keypoints, imgGray, false);
}
else if (detectorType.compare("HARRIS") == 0)
{
    detKeypointsHarris(keypoints, imgGray, false);
}
else 
{
    detKeypointsModern(keypoints, imgGray, detectorType, false);
}
```
---
## MP.3:
Only kept keypoints inside the vehicle rectangle and obtained the average neighbourhood size.

```
double avgNeighSize = 0.;
for (auto& kp : keypoints)
{
    if (vehicleRect.contains(kp.pt))
    {
        kps_vehicle.push_back(kp);
        avgNeighSize += kp.size;
    }
}
keypoints = kps_vehicle;
avgNeighSize /= kps_vehicle.size();
```

---
## MP.4:
Implemented the `BRIEF`, `ORB`, `FREAK`, `AKAZE`, `SIFT`, selectable by string.

```
if (descriptorType.compare("BRIEF") == 0)
{
    extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
}
else if (descriptorType.compare("ORB") == 0)
{
    extractor = cv::ORB::create();
}
else if (descriptorType.compare("FREAK") == 0)
{
    extractor = cv::xfeatures2d::FREAK::create();
}
else if (descriptorType.compare("AKAZE") == 0)
{
    extractor = cv::AKAZE::create();
}
else if (descriptorType.compare("SIFT") == 0)
{
    extractor = cv::xfeatures2d::SIFT::create();
}
```

---
## MP.5

Implement `FLANN` and `KNN` matching algorithms.

FLANN Match:
```
if (matcherType.compare("MAT_FLANN") == 0)
{
    if (descSource.type() != CV_32F)
    { 
        descSource.convertTo(descSource, CV_32F); 
    }
    if (descRef.type() != CV_32F)
    {
        descRef.convertTo(descRef, CV_32F);
    }
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    cout << "FLANN matching";
}
```

KNN Match:
```
if (selectorType.compare("SEL_KNN") == 0)
{ // k nearest neighbors (k=2)
    vector<vector<cv::DMatch>> knnMatch;
    matcher->knnMatch(descSource, descRef, knnMatch, 2); 
}
```

---
## MP.6

Implement the distance ratio in KNN for selecting the best matches.

```
double minNNDistRatio = 0.8;
for (auto it = knnMatch.begin(); it != knnMatch.end(); ++it)
{
    if ((*it)[0].distance < (minNNDistRatio * (*it)[1].distance))
    {
        matches.push_back((*it)[0]);
    }
}
```

---
## MP.7

Detector Performance Evaluation.

| Detector | F0 | F1 | F2| F3 | F4 | F5 | F6 | F7 | F8 | F9 | Total Keypoints | Neighbourhood Size|
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| SHITOMASI | 125|118|123|120|120|113|114|123|111 | 112 | 1179| 4 | 
| HARRIS | 17|14|18|21|26|43|18|31|26|34|248| 6
| FAST | 149|152|150|155|149|149|156|150|138|143|1491 | 7
| BRISK | 264|282|282|277|297|279|289|272|266|254|2762 | 21.94
| ORB | 92|102|106|113|109|125|130|129|127|128|1161 | 56.06
| AKAZE | 166|157|161|155|163|164|173|175|177|179|1670 | 7.71
| SIFT | 138|132|124|137|134|140|137|148|159|137|1386 |5.03


---
## MP.8
Total matches across different combinations.

| Detector/Descriptor | AKAZE | BRIEF | BRISK | FREAK | ORB | SIFT | 
|---|---|---|---|---|---|---|
|AKAZE| 1259 | 1266 | 1215 | 1187 | 1182 | 1270 |
|BRISK| N/A | 1704 | 1570 | 1524 | 1514 | 1646 |
|FAST| N/A | 1099 | 899 | 878 | 1071 | 1046 |
|HARRIS| N/A | 169 | 136 | 140 | 158 | 160 |
|ORB| N/A | 545 | 751 | 420 | 763 | 763 |
|SHITOMASI| N/A | 944 | 767 | 768 | 908 | 927 | 
|SIFT| N/A | 702 | 592 | 593 | N/A | 800 |

---
## MP.9
Avg runtime for different combinations

| Detector/Descriptor | AKAZE | BRIEF | BRISK | FREAK | ORB | SIFT | 
|---|---|---|---|---|---|---|
|AKAZE| 200.7353 | 121.8005 | 401.3959 | 135.1321 | 102.3978 | 126.5250 |
|BRISK| N/A | 320.8763 | 613.2641 | 347.5493 | 323.7865 | 347.7157 |
|FAST| N/A | 3.6889 | 298.0964 | 26.21036 | 1.3944 | 19.2610 |
|HARRIS| N/A | 11.8903 | 302.5134 | 34.9720 | 11.7223 | 21.0356 |
|ORB| N/A | 10.2499 | 300.0926 | 37.1792 | 8.7544 | 60.7399 |
|SHITOMASI| N/A | 16.2889 | 308.3785 | 33.1291 | 15.1846 | 23.3756 | 
|SIFT| N/A | 99.4382 | 352.8828 | 119.3152 | N/A | 144.704 |

### Top 3 detector/descriptor pairs

1. `FAST` + `ORB` : 1071 keypoints in 1.4 ms
2. `FAST` + `BRIEF`: 1099 keypoints in 3.7 ms
3. `ORB` + `ORB`: 763 keypoints in 8.7 ms