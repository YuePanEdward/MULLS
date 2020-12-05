/****************************************************************************
 * Copyright (C) 2019 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * API class for the SemanticKITTI dataset
 ****************************************************************************/

#ifndef __SKDAPI_H__
#define __SKDAPI_H__

namespace skd {

class StampedPose {
public:
    double time_;
    float x_, y_, z_, roll_, pitch_, yaw_;
    std::vector<float> transMat_;
    StampedPose(): x_(0.0f), y_(0.0f), z_(0.0f), roll_(0.0f), pitch_(0.0f), yaw_(0.0f) {};
    StampedPose(double time, float x, float y, float z, float roll, float pitch, float yaw, std::vector<float> transMat):
        time_(time), x_(x), y_(y), z_(z), roll_(roll), pitch_(pitch), yaw_(yaw), transMat_(transMat) {};
    ~StampedPose() {};
};

class VelodynePoint {
public:
    float x_, y_, z_;
    float intensity_;
    unsigned short label_;
    VelodynePoint(): x_(0.0f), y_(0.0f), z_(0.0f), intensity_(0.0f), label_(0) {};
    VelodynePoint(float x, float y, float z, float intensity, unsigned short label): x_(x), y_(y), z_(z), intensity_(intensity), label_(label) {};
    ~VelodynePoint() {};
};

class SKDAPI {
public:
    std::string velodyneDirName_, labelDirName_, calibFilePath_, timestampFilePath_, poseFilePath_;
    std::vector<float> tmVel2Cam_;
    bool areCalibMats_;

    SKDAPI(std::string datasetDirName):
        areCalibMats_(false)
    {
        velodyneDirName_ = datasetDirName + "velodyne/";
        labelDirName_ = datasetDirName + "labels/";
        calibFilePath_ = datasetDirName + "calib.txt";
        timestampFilePath_ = datasetDirName + "times.txt";
        poseFilePath_ = datasetDirName + "poses.txt";
    }

    ~SKDAPI() {};

    std::vector<std::vector<float> > getCalibMats(void) {
        FILE *fp = fopen(calibFilePath_.c_str(), "r");
        if (fp == NULL) {
            fprintf(stderr, "could not open the calibration file -> %s\n", calibFilePath_.c_str());
            exit(1);
        }

        std::vector<std::vector<float> > calibMats;
        for (int i = 0; i < 5; i++) {
            char type[3];
            float m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34;
            fscanf(fp, "%s %f %f %f %f %f %f %f %f %f %f %f %f", type, &m11, &m12, &m13, &m14, &m21, &m22, &m23, &m24, &m31, &m32, &m33, &m34);
            std::vector<float> calibMat = {m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34, 0.0f, 0.0f, 0.0f, 1.0f};
            calibMats.push_back(calibMat);
        }
        fclose(fp);
        tmVel2Cam_ = calibMats[4];
        areCalibMats_ = true;
        return calibMats;
    }

    std::vector<float> getRPY2RotMat(float roll, float pitch, float yaw) {
        float r11 = cosf(pitch) * cosf(yaw);
        float r12 = sinf(roll) * sinf(pitch) * cosf(yaw) - cosf(roll) * sinf(yaw);
        float r13 = cosf(roll) * sinf(pitch) * cosf(yaw) + sinf(roll) * sinf(yaw);
        float r21 = cosf(pitch) * sinf(yaw);
        float r22 = sinf(roll) * sinf(pitch) * sinf(yaw) + cosf(roll) * cosf(yaw);
        float r23 = cosf(roll) * sinf(pitch) * sinf(yaw) - sinf(roll) * cosf(yaw);
        float r31 = -sinf(pitch);
        float r32 = sinf(roll) * cosf(pitch);
        float r33 = cosf(roll) * cosf(pitch);
        std::vector<float> rotMat = {r11, r12, r13, r21, r22, r23, r31, r32, r33};
        return rotMat;
    }

    std::vector<float> convertRotMat2TransMat(std::vector<float> rotMat) {
        std::vector<float> transMat = {rotMat[0], rotMat[1], rotMat[2], 0.0f, rotMat[3], rotMat[4], rotMat[5], 0.0f, rotMat[6], rotMat[7], rotMat[8], 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
        return transMat;
    }

    std::vector<float> getTransMat(float x, float y, float z, float roll, float pitch, float yaw) {
        std::vector<float> rotMat = getRPY2RotMat(roll, pitch, yaw);
        std::vector<float> transMat = {rotMat[0], rotMat[1], rotMat[2], x, rotMat[3], rotMat[4], rotMat[5], y, rotMat[6], rotMat[7], rotMat[8], z, 0.0f, 0.0f, 0.0f, 1.0f};
        return transMat;
    }

    std::vector<float> multiplyTransMats(std::vector<float> mat1, std::vector<float> mat2) {
        std::vector<float> mat(16);
        mat[0] = mat1[0] * mat2[0] + mat1[1] * mat2[4] + mat1[2] * mat2[8] + mat1[3] * mat2[12];
        mat[1] = mat1[0] * mat2[1] + mat1[1] * mat2[5] + mat1[2] * mat2[9] + mat1[3] * mat2[13];
        mat[2] = mat1[0] * mat2[2] + mat1[1] * mat2[6] + mat1[2] * mat2[10] + mat1[3] * mat2[14];
        mat[3] = mat1[0] * mat2[3] + mat1[1] * mat2[7] + mat1[2] * mat2[11] + mat1[3] * mat2[15];
        mat[4] = mat1[4] * mat2[0] + mat1[5] * mat2[4] + mat1[6] * mat2[8] + mat1[7] * mat2[12];
        mat[5] = mat1[4] * mat2[1] + mat1[5] * mat2[5] + mat1[6] * mat2[9] + mat1[7] * mat2[13];
        mat[6] = mat1[4] * mat2[2] + mat1[5] * mat2[6] + mat1[6] * mat2[10] + mat1[7] * mat2[14];
        mat[7] = mat1[4] * mat2[3] + mat1[5] * mat2[7] + mat1[6] * mat2[11] + mat1[7] * mat2[15];
        mat[8] = mat1[8] * mat2[0] + mat1[9] * mat2[4] + mat1[10] * mat2[8] + mat1[11] * mat2[12];
        mat[9] = mat1[8] * mat2[1] + mat1[9] * mat2[5] + mat1[10] * mat2[9] + mat1[11] * mat2[13];
        mat[10] = mat1[8] * mat2[2] + mat1[9] * mat2[6] + mat1[10] * mat2[10] + mat1[11] * mat2[14];
        mat[11] = mat1[8] * mat2[3] + mat1[9] * mat2[7] + mat1[10] * mat2[11] + mat1[11] * mat2[15];
        mat[12] = mat1[12] * mat2[0] + mat1[13] * mat2[4] + mat1[14] * mat2[8] + mat1[15] * mat2[12];
        mat[13] = mat1[12] * mat2[1] + mat1[13] * mat2[5] + mat1[14] * mat2[9] + mat1[15] * mat2[13];
        mat[14] = mat1[12] * mat2[2] + mat1[13] * mat2[6] + mat1[14] * mat2[10] + mat1[15] * mat2[14];
        mat[15] = mat1[12] * mat2[3] + mat1[13] * mat2[7] + mat1[14] * mat2[11] + mat1[15] * mat2[15];
        return mat;
    }

    std::vector<StampedPose> getVelodyneStampedPoses(void) {
        if (!areCalibMats_) {
            fprintf(stderr, "calibration martices are not loaded yet. getCalibMats() must be called before getStampedPoses().\n");
            exit(1);
        }

        std::vector<StampedPose> stampedPoses;
        stampedPoses.clear();

        FILE *fpTime = fopen(timestampFilePath_.c_str(), "r");
        if (fpTime == NULL) {
            fprintf(stderr, "could not open the timestamp file -> %s\n", timestampFilePath_.c_str());
            exit(1);
        }

        FILE *fpPose = fopen(poseFilePath_.c_str(), "r");
        if (fpPose == NULL) {
            fprintf(stderr, "could not open the pose file -> %s\n", poseFilePath_.c_str());
            exit(1);
        }

        // 3x3 rotation matrix will be translated to 4x4 transformation matrix
        std::vector<float> rotMat = convertRotMat2TransMat(getRPY2RotMat(-M_PI / 2.0f, 0.0f, 0.0f));

        float m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34;
        while (fscanf(fpPose, "%f %f %f %f %f %f %f %f %f %f %f %f", &m11, &m12, &m13, &m14, &m21, &m22, &m23, &m24, &m31, &m32, &m33, &m34) != EOF) {
            std::vector<float> camExtMat = {m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34, 0.0f, 0.0f, 0.0f, 1.0f};
            std::vector<float> velMatTmp = multiplyTransMats(camExtMat, tmVel2Cam_);
            std::vector<float> velMat = multiplyTransMats(rotMat, velMatTmp);
            double time;
            fscanf(fpTime, "%lf", &time);
            float x = velMat[3];
            float y = velMat[7];
            float z = velMat[11];
            float roll = atan2(velMat[9], velMat[10]);
            float pitch = asin(-velMat[8]);
            float yaw = atan2(velMat[4], velMat[0]);
            StampedPose stampedPose(time, x, y, z, roll, pitch, yaw, velMat);
            stampedPoses.push_back(stampedPose);
        }
        fclose(fpTime);
        fclose(fpPose);

        if ((int)stampedPoses.size() == 0) {
            fprintf(stderr, "time or pose data could not be loaded -> %s or %s\n", timestampFilePath_.c_str(), poseFilePath_.c_str());
            exit(1);
        }
        return stampedPoses;
    }

    void writeVelodyneTrajectory(std::string trajectoryFilePath, std::vector<StampedPose> stampedPoses) {
        FILE *fp = fopen(trajectoryFilePath.c_str(), "w");
        if (fp == NULL) {
            fprintf(stderr, "trajectory file could not be open -> %s\n", trajectoryFilePath.c_str());
            exit(1);
        }

        for (int i = 0; i < (int)stampedPoses.size(); i++) {
            fprintf(fp, "%lf %f %f %f %f %f %f\n", stampedPoses[i].time_, stampedPoses[i].x_, stampedPoses[i].y_,
                stampedPoses[i].z_, stampedPoses[i].roll_, stampedPoses[i].pitch_, stampedPoses[i].yaw_);
        }
        fclose(fp);
    }

    std::vector<VelodynePoint> getVelodynePoints(int timeIdx) {
        std::vector<VelodynePoint> velodynePoints;
        velodynePoints.clear();

        std::string velodyneFileName, labelFileName;
        if (timeIdx <= 9) {
            velodyneFileName = "00000" + std::to_string(timeIdx) + ".bin";
            labelFileName = "00000" + std::to_string(timeIdx) + ".label";
        } else if (10 <= timeIdx && timeIdx <= 99) {
            velodyneFileName = "0000" + std::to_string(timeIdx) + ".bin";
            labelFileName = "0000" + std::to_string(timeIdx) + ".label";
        } else if (100 <= timeIdx && timeIdx <= 999) {
            velodyneFileName = "000" + std::to_string(timeIdx) + ".bin";
            labelFileName = "000" + std::to_string(timeIdx) + ".label";
        } else {
            velodyneFileName = "00" + std::to_string(timeIdx) + ".bin";
            labelFileName = "00" + std::to_string(timeIdx) + ".label";
        }

        std::string velodyneFilePath = velodyneDirName_ + velodyneFileName;
        std::string labelFilePath = labelDirName_ + labelFileName;
        FILE *fpVelodyne = fopen(velodyneFilePath.c_str(), "r");
        FILE *fpLabel = fopen(labelFilePath.c_str(), "r");
        if (fpVelodyne == NULL || fpLabel == NULL) {
            fprintf(stderr, "The velodyne or label files could not open -> %s, %s\n", velodyneFilePath.c_str(), labelFilePath.c_str());
            return velodynePoints;
        }

        float data[4];
        unsigned short label[2];
        for (;;) {
            size_t velodyneDataSize = fread(data, sizeof(float), 4, fpVelodyne);
            size_t labelDataSize = fread(label, sizeof(unsigned short), 2, fpLabel);
            if (velodyneDataSize == 0 || labelDataSize == 0)
                break;
            VelodynePoint velodynePoint(data[0], data[1], data[2], data[3], label[0]);
            velodynePoints.push_back(velodynePoint);
        }
        fclose(fpVelodyne);
        fclose(fpLabel);

        return velodynePoints;
    }

}; // class SKDAPI

} // namespace skdapi

#endif // __SKDAPI_H_