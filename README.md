# GSLAM-TheiaSfM

## 1. Introduction

This code is the [Theia](http://www.theia-sfm.org/) plugin implementation base on [GSLAM](https://github.com/zdzhaoyong/GSLAM).

![GSLAM-TheiaSfM](./data/images/gslam_theiaSfM.png)

## 2. Build and Install
### 2.1. Build and Install GSLAM

git clone https://github.com/zdzhaoyong/GSLAM --branch 2.4.2

### 2.2. Build and Install GSLAM-TheiaSfM

```
mkdir build;
cd build;
cmake .. -DCMAKE_BUILD_TYPE=Release;
make;
sudo make install
```

## 3. Run TheiaSfM with gslam
Since TheiaSfM plugin now use image file path as input, the dataset should provide frames with filePath information.

Now DroneMap keyframes datasets and RTMapper datasets are supported.

1. Download sample dataset:

```
git clone https://github.com/zdzhaoyong/phantom3-village-kfs
```

2. Run with gslam

```
gslam SLAM=libgslam_theiaSfM Dataset=<folder>/phantom3-village-kfs/.npudronemap
```


Since the SfM process is quite slow compare to SLAM plugins, the real process is started after all frames feeded. We suggest users to try a small dataset first or stop feeding frames with pressing stop button when enough frames are feeded. You can further boost the speed with parameter PlaySpeed and num_threads setted.


```
gslam SLAM=libgslam_theiaSfM Dataset=<folder>/phantom3-village-kfs/.npudronemap PlaySpeed=5 num_threads=4
```
