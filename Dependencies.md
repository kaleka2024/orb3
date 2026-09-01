# List of Known Dependencies

## ORB-SLAM3 v1.0

In this document we list all the pieces of code included by ORB-SLAM3 and linked libraries which are not property of the authors of ORB-SLAM3.

### Code in **src** and **include** folders

* *ORBextractor.cc*.
This is a modified version of `orb.cpp` of OpenCV library. The original code is BSD licensed.

* *MLPnPsolver.h, MLPnPsolver.cc*.
This is a modified version of the MLPnP of Steffen Urban from [here](https://github.com/urbste/opengv).
The original code is BSD licensed.

* Function *ORBmatcher::DescriptorDistance* in *ORBmatcher.cc*.
The code is from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel.
The code is in the public domain.

#####Code in Thirdparty folder

* All code in **DBoW2** folder.
This is a modified version of [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib) library. All files included are BSD licensed.
