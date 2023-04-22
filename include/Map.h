/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "MapDrawer.h"
#include "gurobi_c++.h"

#include <malloc.h>
#include <sys/stat.h>
#include <set>
#include <unordered_map>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace std {
    template<>
    struct less<Eigen::Vector3i> {
    public:
        bool operator()(Eigen::Vector3i const &a, Eigen::Vector3i const &b) const {
            for (size_t i = 0; i < a.size(); ++i) {
                if (a[i] < b[i]) return true;
                if (a[i] > b[i]) return false;
            }
            return false;
        }
    };

    template<>
    struct less<Eigen::Vector2i> {
    public:
        bool operator()(Eigen::Vector2i const &a, Eigen::Vector2i const &b) const {
            for (size_t i = 0; i < a.size(); ++i) {
                if (a[i] < b[i]) return true;
                if (a[i] > b[i]) return false;
            }
            return false;
        }
    };
}

namespace ORB_SLAM2 {

    class MapPoint;

    class MapDrawer;

    class KeyFrame;

    class KeyFrameDatabase;

    class Map {
    public:
        Map(KeyFrameDatabase *pKFDB, const ORBVocabulary &voc, const string &strSettingPath);

        void SetDrawer(MapDrawer *pMapDrawer);

        void AddKeyFrame(KeyFrame *pKF);

        void AddMapPoint(MapPoint *pMP);

        void EraseMapPoint(MapPoint *pMP);

        void EraseKeyFrame(KeyFrame *pKF);

        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

        void InformNewBigChange();

        int GetLastBigChangeIdx();

        std::vector<KeyFrame *> GetAllKeyFrames();

        std::vector<MapPoint *> GetAllMapPoints();

        std::vector<MapPoint *> GetReferenceMapPoints();

        long unsigned int MapPointsInMap();

        long unsigned KeyFramesInMap();

        long unsigned int GetMaxKFid();

        void clear();

        bool Save(const string &filename);

        bool Load(const string &filename, ORBVocabulary *pVoc, bool bReference = false);

        bool LoadCompressed(const string &filename, ORBVocabulary *pVoc);

        void CompressByGrid2D();

        void CompressByGrid3D();

        void DeleteCompressedMPs();

        void ConstructCubeMap();

        // This is to load the camera parameter and later used for retrieving the map
        bool LoadCofficient(const string &strSettingPath);

        vector<KeyFrame *> mvpKeyFrameOrigins;

        std::mutex mMutexMapUpdate;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;

        int mnMinU;
        int mnMinV;
        int mnMaxU;
        int mnMaxV;
    protected:
        static void SaveMapPoint(ofstream &f, MapPoint *mp);

        static void SaveKeyFrame(ofstream &f, KeyFrame *kf);

        MapPoint *ReadMapPoint(ifstream &f);

        KeyFrame *
        ReadKeyFrame(ifstream &f, ORBVocabulary *pVoc, ORBextractor *ex);

        std::set<MapPoint *> mspMapPoints;
        std::set<KeyFrame *> mspKeyFrames;

        std::vector<MapPoint *> mvpReferenceMapPoints;

        std::vector<Eigen::Vector3f> mvSpacePoints;

        std::vector<Eigen::Vector3f> mvSpacePointsReference;

        long unsigned int mnMaxKFid;

        // Index related to a big change in the map (loop closure, global BA)
        int mnBigChangeIdx;

        std::mutex mMutexMap;

        KeyFrameDatabase *mpKeyFrameDB;

        MapDrawer *mpMapDrawer;

        // Associated vocabulary
        const ORBVocabulary *mpORBVocabulary;

        // Calibration matrix and OpenCV distortion parameters.
        cv::Mat mK;
        float fx;
        float fy;
        float cx;
        float cy;
        float invfx;
        float invfy;
        cv::Mat mDistCoef;
        float mbf;

        int nFeatures;
        float fScaleFactor;
        int nLevels;
        int fIniThFAST;
        int fMinThFAST;

        std::map<long unsigned int, KeyFrame *> mID_KFs;

        float mfResolution;

        std::map<Eigen::Vector3i, std::set<long unsigned int>> mCubeMap;
        std::map<Eigen::Vector3i, std::set<long unsigned int>> mCubeMapReference;
        int mnCubeNum;

        string mstrSettingPath;

        std::map<unsigned long int, MapPoint *> mID_MPs;
    };

} //namespace ORB_SLAM

#endif // MAP_H
