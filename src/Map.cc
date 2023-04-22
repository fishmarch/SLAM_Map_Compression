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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2 {
    Map::Map(KeyFrameDatabase *pKFDB, const ORBVocabulary &voc, const string &strSettingPath) :
            mnMaxKFid(0), mpORBVocabulary(&voc), mnBigChangeIdx(0), mpKeyFrameDB(pKFDB),
            mstrSettingPath(strSettingPath) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mnMinU = 0;
        mnMinV = 0;
        mnMaxU = fSettings["Camera.width"];
        mnMaxV = fSettings["Camera.height"];
    }

    void Map::SetDrawer(MapDrawer *pMapDrawer) {
        mpMapDrawer = pMapDrawer;
    }

    void Map::AddKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.insert(pKF);
        if (pKF->mnId > mnMaxKFid)
            mnMaxKFid = pKF->mnId;
    }

    void Map::AddMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void Map::EraseMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::EraseKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs) {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange() {
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;
    }

    int Map::GetLastBigChangeIdx() {
        unique_lock<mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    vector<KeyFrame *> Map::GetAllKeyFrames() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    }

    vector<MapPoint *> Map::GetAllMapPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    }

    long unsigned int Map::MapPointsInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    vector<MapPoint *> Map::GetReferenceMapPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetMaxKFid() {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void Map::clear() {
        for (set<MapPoint *>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++)
            delete *sit;

        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
            delete *sit;

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    bool Map::Save(const string &filename) {
        cout << "Map: Saving to " << filename << endl;
        ofstream f;
        f.open(filename.c_str(), ios_base::out | ios::binary);

        const vector<MapPoint *> vpMPs = GetAllMapPoints();
        for (MapPoint *mp: vpMPs) {
            if (mp->isBad() || mp->Observations() == 0) {
                EraseMapPoint(mp);
            }
        }

        cout << "  saving " << mspMapPoints.size() << " mappoints ...";
        unsigned long int nbMapPoints = mspMapPoints.size();
        f.write((char *) &nbMapPoints, sizeof(nbMapPoints));
        for (MapPoint *mp: mspMapPoints)
            SaveMapPoint(f, mp);

        cout << " Done!" << endl;

        cout << "  saving " << mspKeyFrames.size() << " keyframes ...";
        unsigned long int nbKeyFrames = mspKeyFrames.size();
        f.write((char *) &nbKeyFrames, sizeof(nbKeyFrames));
        for (KeyFrame *kf: mspKeyFrames)
            SaveKeyFrame(f, kf);

        cout << " Done!" << endl;

        // store tree and graph
        for (KeyFrame *kf: mspKeyFrames) {
            KeyFrame *parent = kf->GetParent();
            unsigned long int parent_id = ULONG_MAX;
            if (parent)
                parent_id = parent->mnId;
            f.write((char *) &parent_id, sizeof(parent_id));
            unsigned long int nb_con = kf->GetConnectedKeyFrames().size();
            f.write((char *) &nb_con, sizeof(nb_con));
            for (KeyFrame *ckf: kf->GetConnectedKeyFrames()) {
                int weight = kf->GetWeight(ckf);
                f.write((char *) &ckf->mnId, sizeof(ckf->mnId));
                f.write((char *) &weight, sizeof(weight));
            }
        }

        f.close();
        cout << "Map: finished saving" << endl;
        struct stat st;
        stat(filename.c_str(), &st);
        cout << "Map: saved " << st.st_size << " bytes" << endl;

        return true;
    }

    void Map::SaveMapPoint(ofstream &f, MapPoint *mp) {
        f.write((char *) &mp->mnId, sizeof(mp->mnId));
        cv::Mat wp = mp->GetWorldPos();
        f.write((char *) &wp.at<float>(0), sizeof(float));
        f.write((char *) &wp.at<float>(1), sizeof(float));
        f.write((char *) &wp.at<float>(2), sizeof(float));
        f.write((char *) &mp->mnVisible, sizeof(int));
        f.write((char *) &mp->mnFound, sizeof(int));

        float fMaxDis = mp->GetMaxDistanceInvariance() / 1.2;
        float fMinDis = mp->GetMinDistanceInvariance() / 0.8;
        float fMaxDegree = 0.5236; // 30 degrees

        cv::Mat Pos = wp;

        KeyFrame *pRefKF = mp->GetReferenceKeyFrame();

        f.write((char *) &pRefKF->mnId, sizeof(pRefKF->mnId));

        cv::Mat PC = Pos - pRefKF->GetCameraCenter();
        float dist = cv::norm(PC);
        fMaxDis = fMaxDis < (dist * 1.44) ? fMaxDis : (dist * 1.44);
        fMinDis = fMinDis > (dist * 0.69) ? fMinDis : (dist * 0.69);

        cv::Mat normal = mp->GetNormal();
        map<KeyFrame *, size_t> observations = mp->GetObservations();
        float fAveDis = 0.0;
        float fAveDegree = 0.0;
        vector<float> vDis;
        vector<float> vDegree;
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKF = mit->first;
            PC = Pos - pKF->GetCameraCenter();
            dist = cv::norm(PC);
            float viewCos = PC.dot(normal) / dist;
            float viewAngle = acos(viewCos);

            fAveDegree += viewAngle;
            fAveDis += dist;
            vDis.push_back(dist);
            vDegree.push_back(viewAngle);
        }
        fAveDis /= observations.size();
        fAveDegree /= observations.size();
        float fvarDis = 0.0;
        float fvarDegree = 0.0;
        for (int i = 0; i < vDis.size(); ++i) {
            fvarDis += pow(vDis[i] - fAveDis, 2);
            fvarDegree += pow(vDegree[i] - fAveDegree, 2);
        }
        fvarDis = sqrt(fvarDis / vDis.size());
        fvarDegree = sqrt(fvarDegree / vDis.size());

        fMaxDis = fMaxDis > (fAveDis + fvarDis) ? fMaxDis : (fAveDis + fvarDis);
        fMinDis = fMinDis < (fAveDis - fvarDis) ? fMinDis : (fAveDis - fvarDis);
        fMaxDegree = fMaxDegree > (fAveDegree + fvarDegree) ? fMaxDegree : (fAveDegree + fvarDegree);

        f.write((char *) &fMaxDis, sizeof(float));
        f.write((char *) &fMinDis, sizeof(float));
        f.write((char *) &fMaxDegree, sizeof(float));
    }

    void Map::SaveKeyFrame(ofstream &f, KeyFrame *kf) {
        f.write((char *) &kf->mnId, sizeof(kf->mnId));
        f.write((char *) &kf->mnFrameId, sizeof(kf->mnFrameId));
        f.write((char *) &kf->mTimeStamp, sizeof(kf->mTimeStamp));

        cv::Mat Tcw = kf->GetPose();
        f.write((char *) &Tcw.at<float>(0, 3), sizeof(float));
        f.write((char *) &Tcw.at<float>(1, 3), sizeof(float));
        f.write((char *) &Tcw.at<float>(2, 3), sizeof(float));
        vector<float> Qcw = Converter::toQuaternion(Tcw.rowRange(0, 3).colRange(0, 3));
        f.write((char *) &Qcw[0], sizeof(float));
        f.write((char *) &Qcw[1], sizeof(float));
        f.write((char *) &Qcw[2], sizeof(float));
        f.write((char *) &Qcw[3], sizeof(float));

        int nMPs = 0;
        for (int i = 0; i < kf->N; i++) {
            MapPoint *mp = kf->GetMapPoint(i);
            if (mp != nullptr)
                nMPs++;
        }
        f.write((char *) &nMPs, sizeof(nMPs));
        for (int i = 0; i < kf->N; i++) {
            MapPoint *mp = kf->GetMapPoint(i);
            if (mp == nullptr)
                continue;
            cv::KeyPoint kp = kf->mvKeys[i];
            f.write((char *) &kp.pt.x, sizeof(kp.pt.x));
            f.write((char *) &kp.pt.y, sizeof(kp.pt.y));
            f.write((char *) &kp.size, sizeof(kp.size));
            f.write((char *) &kp.angle, sizeof(kp.angle));
            f.write((char *) &kp.response, sizeof(kp.response));
            f.write((char *) &kp.octave, sizeof(kp.octave));
            for (int j = 0; j < 32; j++)
                f.write((char *) &kf->mDescriptors.at<unsigned char>(i, j), sizeof(char));

            unsigned long int mpidx;
            mpidx = mp->mnId;
            f.write((char *) &mpidx, sizeof(mpidx));
        }

    }

    bool Map::Load(const string &filename, ORBVocabulary *pVoc, bool bReference) {

        ORB_SLAM2::ORBextractor orb_ext = ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST,
                                                                  fMinThFAST);

        cout << "Map: loading from " << filename << "..." << endl;
        ifstream f;
        f.open(filename.c_str());

        long unsigned int nb_mappoints, max_id = 0;
        f.read((char *) &nb_mappoints, sizeof(nb_mappoints));
        cout << "Loading " << nb_mappoints << " mappoints ...";
        mvpReferenceMapPoints.clear();
        for (unsigned int i = 0; i < nb_mappoints; i++) {
            ORB_SLAM2::MapPoint *mp = ReadMapPoint(f);
            if (mp->mnId >= max_id)
                max_id = mp->mnId;
            AddMapPoint(mp);
            mID_MPs[mp->mnId] = mp;
            if (bReference)
                mvpReferenceMapPoints.push_back(mp);
        }
        ORB_SLAM2::MapPoint::nNextId = max_id + 1;
        cout << " Done!" << endl;

        long unsigned int nb_keyframes;
        f.read((char *) &nb_keyframes, sizeof(nb_keyframes));
        cout << "Loading " << nb_keyframes << " keyframes ...";

        vector<KeyFrame *> KFs_by_order;
        for (unsigned int i = 0; i < nb_keyframes; i++) {
            KeyFrame *kf = ReadKeyFrame(f, pVoc, &orb_ext);
            kf->ComputeBoW();
            AddKeyFrame(kf);
            mpKeyFrameDB->add(kf);
            mID_KFs[kf->mnId] = kf;
            KFs_by_order.push_back(kf);
        }
        Frame::nNextId++;
        cout << " Done!" << endl;

        cout << "Loading Spanning tree ...";
        // Load Spanning tree
        for (KeyFrame *kf: KFs_by_order) {
            unsigned long int parent_id;
            f.read((char *) &parent_id, sizeof(parent_id));
            if (parent_id != ULONG_MAX && mID_KFs[parent_id])
                kf->ChangeParent(mID_KFs[parent_id]);
            unsigned long int nb_con;
            f.read((char *) &nb_con, sizeof(nb_con));
            for (unsigned long int i = 0; i < nb_con; i++) {
                unsigned long int id;
                int weight;
                f.read((char *) &id, sizeof(id));
                f.read((char *) &weight, sizeof(weight));
                if (mID_KFs[id])
                    kf->AddConnection(mID_KFs[id], weight);
            }
        }
        cout << " Done!" << endl;

        const vector<MapPoint *> vpMPs = GetAllMapPoints();
        // MapPoints descriptors
        for (MapPoint *pMP: vpMPs) {
            if (!pMP || pMP->mbEmpty) {
                EraseMapPoint(pMP);
                continue;
            }
            map<KeyFrame *, size_t> observations = pMP->GetObservations();

            KeyFrame *pKF = mID_KFs[pMP->mnRefKFID];
            if (!pKF) {
                pKF = observations.begin()->first;
            }
            pMP->SetReferenceKeyFrame(pKF);
            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            float err = 0;
            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
                KeyFrame *pKFi = mit->first;
                float err_i = pKFi->ReProjectionError(mit->second);
                err += err_i;
            }
            err = sqrt(err / observations.size());
            pMP->mfErr = err;

            cv::Mat Pos = pMP->GetWorldPos();
            cv::Mat normal = pMP->GetNormal();
            float fMaxDis = 0.0;
            float fMinDis = 500.0;
            float fMaxDegree = 0.0;
            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
                KeyFrame *pKF = mit->first;
                cv::Mat PC = Pos - pKF->GetCameraCenter();
                float dist = cv::norm(PC);
                float viewCos = PC.dot(normal) / dist;
                float viewAngle = acos(viewCos);
                if (dist > fMaxDis)
                    fMaxDis = dist;
                if (dist < fMinDis)
                    fMinDis = dist;
                if (viewAngle > fMaxDegree)
                    fMaxDegree = viewAngle;
            }
            pMP->mfMaxDis = fMaxDis;
            pMP->mfMinDis = fMinDis;
            pMP->mfMaxDegree = fMaxDegree;

        }
        cout << "Map loaded! " << endl;

        return true;
    }

    bool Map::LoadCompressed(const string &filename, ORBVocabulary *pVoc) {

        cout << "Map: reading from " << filename << "..." << endl;
        ifstream f;
        f.open(filename.c_str());

        long unsigned int nb_mappoints;
        f.read((char *) &nb_mappoints, sizeof(nb_mappoints));
        cout << "Loading " << nb_mappoints << " mappoints ...";
        mvpReferenceMapPoints.clear();
        for (unsigned int i = 0; i < nb_mappoints; i++) {
            ORB_SLAM2::MapPoint *mp = ReadMapPoint(f);
            if (mID_MPs[mp->mnId]) {
                unique_lock<mutex> lock(mMutexMap);
                mvpReferenceMapPoints.push_back(mID_MPs[mp->mnId]);
            }
        }
        return true;
    }

    MapPoint *Map::ReadMapPoint(ifstream &f) {
        long unsigned int id;
        f.read((char *) &id, sizeof(id));
        cv::Mat wp(3, 1, CV_32F);
        f.read((char *) &wp.at<float>(0), sizeof(float));
        f.read((char *) &wp.at<float>(1), sizeof(float));
        f.read((char *) &wp.at<float>(2), sizeof(float));
        long int mnFirstKFid = 0, mnFirstFrame = 0;
        int visible, found;
        f.read((char *) &visible, sizeof(int));
        f.read((char *) &found, sizeof(int));
        MapPoint *mp = new MapPoint(wp, mnFirstKFid, mnFirstFrame, this);
        mp->mnId = id;
        mp->mnVisible = visible;
        mp->mnFound = found;

        long unsigned int KFid;
        f.read((char *) &KFid, sizeof(long unsigned int));
        mp->mnRefKFID = KFid;

        float fMaxDis, fMinDis, fMaxDegree;
        f.read((char *) &fMaxDis, sizeof(float));
        f.read((char *) &fMinDis, sizeof(float));
        f.read((char *) &fMaxDegree, sizeof(float));
        mp->mfMaxDis = fMaxDis;
        mp->mfMinDis = fMinDis;
        mp->mfMaxDegree = fMaxDegree;
        return mp;
    }

    KeyFrame *
    Map::ReadKeyFrame(ifstream &f, ORBVocabulary *pVoc, ORBextractor *orb_ext) {
        Frame fr;
        fr.mpORBvocabulary = pVoc;
        long unsigned int kfID;
        f.read((char *) &kfID, sizeof(kfID));
        f.read((char *) &fr.mnId, sizeof(fr.mnId));
        f.read((char *) &fr.mTimeStamp, sizeof(fr.mTimeStamp));
        cv::Mat Tcw(4, 4, CV_32F);
        f.read((char *) &Tcw.at<float>(0, 3), sizeof(float));
        f.read((char *) &Tcw.at<float>(1, 3), sizeof(float));
        f.read((char *) &Tcw.at<float>(2, 3), sizeof(float));
        Tcw.at<float>(3, 3) = 1.;
        cv::Mat Qcw(1, 4, CV_32F);
        f.read((char *) &Qcw.at<float>(0, 0), sizeof(float));
        f.read((char *) &Qcw.at<float>(0, 1), sizeof(float));
        f.read((char *) &Qcw.at<float>(0, 2), sizeof(float));
        f.read((char *) &Qcw.at<float>(0, 3), sizeof(float));
        Converter::RmatOfQuat(Tcw, Qcw);
        fr.SetPose(Tcw);
        f.read((char *) &fr.N, sizeof(fr.N));
        fr.mvKeys.reserve(fr.N);
        fr.mDescriptors.create(fr.N, 32, CV_8UC1);
        fr.mvpMapPoints = vector<MapPoint *>(fr.N, static_cast<MapPoint *>(NULL));

        for (int i = 0; i < fr.N; i++) {
            cv::KeyPoint kp;
            f.read((char *) &kp.pt.x, sizeof(kp.pt.x));
            f.read((char *) &kp.pt.y, sizeof(kp.pt.y));
            f.read((char *) &kp.size, sizeof(kp.size));
            f.read((char *) &kp.angle, sizeof(kp.angle));
            f.read((char *) &kp.response, sizeof(kp.response));
            f.read((char *) &kp.octave, sizeof(kp.octave));
            fr.mvKeys.push_back(kp);
            for (int j = 0; j < 32; j++)
                f.read((char *) &fr.mDescriptors.at<unsigned char>(i, j), sizeof(char));
            unsigned long int mpidx;
            f.read((char *) &mpidx, sizeof(mpidx));
            if (mpidx == ULONG_MAX)
                fr.mvpMapPoints[i] = NULL;
            else
                fr.mvpMapPoints[i] = mID_MPs[mpidx];
        }

        // load camera parameters
        fr.fx = fx;
        fr.fy = fy;
        fr.cx = cx;
        fr.cy = cy;
        fr.invfx = invfx;
        fr.invfy = invfy;
        fr.mDistCoef = mDistCoef.clone();
        fr.mK = mK.clone();

        fr.mvuRight = vector<float>(fr.N, -1);
        fr.mvDepth = vector<float>(fr.N, -1);
        fr.mpORBextractorLeft = orb_ext;
        fr.InitializeScaleLevels();
        fr.UndistortKeyPoints();
        fr.AssignFeaturesToGrid();
        fr.ComputeBoW();

        KeyFrame *kf = new KeyFrame(fr, this, mpKeyFrameDB);
        kf->mnId = kfID;
        Frame::nNextId = Frame::nNextId > fr.mnId ? Frame::nNextId : fr.mnId;
        for (int i = 0; i < fr.N; i++) {
            if (fr.mvpMapPoints[i]) {
                fr.mvpMapPoints[i]->AddObservation(kf, i);
            }
        }
        return kf;
    }

    bool Map::LoadCofficient(const string &strSettingPath) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        fx = fSettings["Camera.fx"];
        fy = fSettings["Camera.fy"];
        cx = fSettings["Camera.cx"];
        cy = fSettings["Camera.cy"];
        invfx = 1.0 / fx;
        invfy = 1.0 / fy;
        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);
        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0) {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);
        mbf = fSettings["Camera.bf"];

        nFeatures = fSettings["ORBextractor.nFeatures"];
        fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        nLevels = fSettings["ORBextractor.nLevels"];
        fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        fMinThFAST = fSettings["ORBextractor.minThFAST"];

        return true;
    }

    void Map::CompressByGrid2D() {
        cout << "------------------" << endl;
        cout << "Map Size: " << mspMapPoints.size() << " before compression" << endl;
        cout << "------------------" << endl;

        cv::FileStorage fSettings(mstrSettingPath, cv::FileStorage::READ);
        const int b = fSettings["Map.CompressByGrid2D.num"]; // minimum number of features in every keyframe.

        // Create an environment
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "mip1.log");
        env.start();
        GRBModel model = GRBModel(env);

        // Set optimization parameters
        int maxObservations = 0;
        float aveObservations = 0;
        float maxVolume = 0.0;
        float aveVolume = 0.0;
        GRBVar x[mspMapPoints.size()];
        int index = 0;
        for (MapPoint *pMP: mspMapPoints) {
            pMP->mnIndexForCompression = index;
            x[index] = model.addVar(0, 1, 0, GRB_BINARY);
            index++;
            if (maxObservations < pMP->Observations())
                maxObservations = pMP->Observations();
        }

        // Set optimization objective
        GRBLinExpr expr_objective = 0.0;
        index = 0;
        for (MapPoint *pMP: mspMapPoints) {
            float coef = (float) maxObservations - pMP->Observations();
            expr_objective += coef * x[index];
            index++;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//        float fGridWidth = fSettings["Map.CompressByScore.GridWidth"];
//        fGridWidth = 1.0 / fGridWidth;
        float fGridX = fSettings["Map.CompressByGrid2D.GridX"];
        float fGridY = fSettings["Map.CompressByGrid2D.GridY"];
        float fCenterX = fGridX / 2.0;
        float fCenterY = fGridY / 2.0;
        float ddMax = sqrt(fCenterX * fCenterX + fCenterY * fCenterY);

        float fGridWidthX = (float) mnMaxU / fGridX;
        float fGridWidthY = (float) mnMaxV / fGridY;
        fGridWidthX = 1.0 / fGridWidthX;
        fGridWidthY = 1.0 / fGridWidthY;

        float lambda_grid = fSettings["Map.CompressByGrid2D.lambda2"];
        float lambda = fSettings["Map.CompressByGrid2D.lambda"];

        for (KeyFrame *pKF: mspKeyFrames) {
            std::map<Eigen::Vector2i, std::set<long unsigned int>> grids;
            set<MapPoint *> sMPs = pKF->GetMapPoints();
            GRBLinExpr expr_score = 0.0;
            for (MapPoint *pMP: sMPs) {
                int gridx, gridy;
                cv::Mat pos = pKF->GetProjectionPosition(pMP);

                gridx = pos.at<float>(0) * fGridWidthX;
                gridy = pos.at<float>(1) * fGridWidthY;
                Eigen::Vector2i grid = Eigen::Vector2i(gridx, gridy);
                if (!grids.count(grid)) {
                    grids[grid] = set<long unsigned int>();
                }
                grids[grid].insert(pMP->mnIndexForCompression);

                expr_score += x[pMP->mnIndexForCompression];
            }

            for (const auto &g: grids) {

                set<long unsigned int> indexes = g.second;
                GRBVar th = model.addVar(0, 1, 0, GRB_BINARY);
                GRBLinExpr expr_cons = 0;
                GRBLinExpr expr_cons2 = 1 - th;
                for (long unsigned int i: indexes) {
                    expr_cons += x[i];
                }
                model.addConstr(expr_cons, '>', expr_cons2);
                expr_objective += lambda_grid * th;
            }

            GRBVar th = model.addVar(0, b, 0, GRB_INTEGER);
            expr_score += th;
            expr_objective += lambda * th;
            model.addConstr(expr_score, '>', b);
        }
        model.setObjective(expr_objective, GRB_MINIMIZE);
        float MIPGap = fSettings["Map.CompressByGrid2D.MIPGap"];
        if (MIPGap > 0)
            model.set(GRB_DoubleParam_MIPGap, MIPGap);

        model.optimize();
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double tt = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        cout << "------------------" << endl;
        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
        cout << "Time: " << tt << " s" << endl;
        cout << "------------------" << endl;

        mvpReferenceMapPoints.clear();

        for (MapPoint *pMP: mspMapPoints) {
            long unsigned int i = pMP->mnIndexForCompression;
            double keep = x[i].get(GRB_DoubleAttr_X);
            if (keep > 0) {
                unique_lock<mutex> lock(mMutexMap);
                mvpReferenceMapPoints.push_back(pMP);
                pMP->SetRemained(true);
            } else {
                pMP->SetRemained(false);
            }
        }
    }

    void Map::CompressByGrid3D() {
        ConstructCubeMap();
        cout << "------------------" << endl;
        cout << "Map Size: " << mspMapPoints.size() << " before compression" << endl;
        cout << "------------------" << endl;

        cv::FileStorage fSettings(mstrSettingPath, cv::FileStorage::READ);
        const int b = fSettings["Map.CompressByGrid3D.num"]; // minimum number of features in every keyframe.

        // Create an environment
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "mip1.log");
        env.start();
        GRBModel model = GRBModel(env);

        // Set optimization parameters
        int maxObservations = 0;
        float aveObservations = 0;
        float maxVolume = 0.0;
        float aveVolume = 0.0;
        GRBVar x[mspMapPoints.size()];
        int index = 0;
        for (MapPoint *pMP: mspMapPoints) {
            pMP->mnIndexForCompression = index;
            x[index] = model.addVar(0, 1, 0, GRB_BINARY);
            index++;
            if (maxObservations < pMP->Observations())
                maxObservations = pMP->Observations();

        }

        // Set optimization objective
        GRBLinExpr expr_objective = 0.0;
        index = 0;
        for (MapPoint *pMP: mspMapPoints) {
            float coef = (float) maxObservations - pMP->Observations();
            expr_objective += coef * x[index];
            index++;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//        float fGridWidth = fSettings["Map.CompressByScore.GridWidth"];
//        fGridWidth = 1.0 / fGridWidth;
        float fGridX = fSettings["Map.CompressByGrid3D.GridX"];
        float fGridY = fSettings["Map.CompressByGrid3D.GridY"];
        float fCenterX = fGridX / 2.0;
        float fCenterY = fGridY / 2.0;
        float ddMax = sqrt(fCenterX * fCenterX + fCenterY * fCenterY);
        float fGridWidthX = (float) mnMaxU / fGridX;
        float fGridWidthY = (float) mnMaxV / fGridY;
        fGridWidthX = 1.0 / fGridWidthX;
        fGridWidthY = 1.0 / fGridWidthY;

        float lambda_grid = fSettings["Map.CompressByGrid3D.lambda2"];
        float lambda = fSettings["Map.CompressByGrid3D.lambda"];

        for (KeyFrame *pKF: mspKeyFrames) {
            std::map<Eigen::Vector2i, std::set<long unsigned int>> grids;
            set<MapPoint *> sMPs = pKF->GetMapPoints();
            GRBLinExpr expr_score = 0.0;
            for (MapPoint *pMP: sMPs) {
                int gridx, gridy;
                cv::Mat pos = pKF->GetProjectionPosition(pMP);

                gridx = pos.at<float>(0) * fGridWidthX;
                gridy = pos.at<float>(1) * fGridWidthY;
                Eigen::Vector2i grid = Eigen::Vector2i(gridx, gridy);
                if (!grids.count(grid)) {
                    grids[grid] = set<long unsigned int>();
                }
                grids[grid].insert(pMP->mnIndexForCompression);
                expr_score += x[pMP->mnIndexForCompression];
            }

            if (lambda_grid > 0) {
                for (const auto &g: grids) {
                    set<long unsigned int> indexes = g.second;
                    GRBVar th = model.addVar(0, 1, 0, GRB_BINARY);
                    GRBLinExpr expr_cons = 0;
                    GRBLinExpr expr_cons2 = 1 - th;
                    for (long unsigned int i: indexes) {
                        expr_cons += x[i];
                    }
                    model.addConstr(expr_cons, '>', expr_cons2);
                    expr_objective += lambda_grid * th;
                }
            }

            GRBVar th = model.addVar(0, b, 0, GRB_INTEGER);
            expr_score += th;
            expr_objective += lambda * th;
            model.addConstr(expr_score, '>', b);
        }

        float lambda_space = fSettings["Map.CompressByGrid3D.lambda3"];
        int b2 = fSettings["Map.CompressByGrid3D.num2"];

        for (auto cube: mCubeMap) {
            std::set<long unsigned int> sMPInCube = cube.second;
            if (sMPInCube.size() < b2)
                continue;
            GRBLinExpr expr_cons = 0;
            for (long unsigned int id: sMPInCube) {
                MapPoint *pMP = mID_MPs[id];
                expr_cons += x[pMP->mnIndexForCompression];
            }
            GRBVar th = model.addVar(0, b2, 0, GRB_INTEGER);
            expr_objective += lambda_space * th;
            expr_cons += th;
            model.addConstr(expr_cons, '>', b2);
        }
        mCubeMap.clear();
        cout << "malloc_trim " << malloc_trim(0) << endl;

        model.setObjective(expr_objective, GRB_MINIMIZE);
        float MIPGap = fSettings["Map.CompressByGrid3D.MIPGap"];
        if (MIPGap > 0)
            model.set(GRB_DoubleParam_MIPGap, MIPGap);

        model.optimize();

        mvpReferenceMapPoints.clear();

        for (MapPoint *pMP: mspMapPoints) {
            long unsigned int i = pMP->mnIndexForCompression;
            double keep = x[i].get(GRB_DoubleAttr_X);
            if (keep > 0) {
                unique_lock<mutex> lock(mMutexMap);
                mvpReferenceMapPoints.push_back(pMP);
                pMP->SetRemained(true);
            } else {
                pMP->SetRemained(false);
            }
        }
    }

    void Map::DeleteCompressedMPs() {
        vector<MapPoint *> vpMPs = GetAllMapPoints();
        for (MapPoint *pMP: vpMPs) {
            if (pMP->isBad())
                continue;
            if (!pMP->IsRemained()) {
                pMP->SetBadFlag();
            }
        }
    }

    void Map::ConstructCubeMap() {
        mnCubeNum = 0;
        cv::FileStorage fSettings(mstrSettingPath, cv::FileStorage::READ);
        mfResolution = fSettings["Map.Resolution"];
        int b2 = fSettings["Map.CompressByGrid3D.num2"];

        vector<MapPoint *> vpMapPoints;
        vpMapPoints = GetAllMapPoints();
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        for (int i = 0; i < vpMapPoints.size(); ++i) {
            MapPoint *pMP = vpMapPoints[i];
            if (pMP->isBad() || !pMP->GetReferenceKeyFrame())
                continue;

            int nRefKF = pMP->GetReferenceKeyFrame()->mnId;
            cv::Mat Pos = pMP->GetWorldPos();
            cv::Mat normal = pMP->GetNormal();
            float fMaxDis = pMP->mfMaxDis;
            float fMinDis = pMP->mfMinDis;
            float fMaxDegree = pMP->mfMaxDegree;

            if (fMinDis > 50)
                continue;
            cv::Mat a = (cv::Mat_<float>(3, 1) << 0, normal.at<float>(2), -normal.at<float>(1));
            float a_dist = cv::norm(a);
            if (a_dist < 0.00001) {
                a.at<float>(0) = normal.at<float>(1);
                a.at<float>(1) = -normal.at<float>(0);
                a.at<float>(2) = 0;
            }
            a = a / cv::norm(a);
            cv::Mat b = (cv::Mat_<float>(3, 1) <<
                                               normal.at<float>(1) * a.at<float>(2) -
                                               normal.at<float>(2) * a.at<float>(1),
                    normal.at<float>(2) * a.at<float>(0) - normal.at<float>(0) * a.at<float>(2),
                    normal.at<float>(0) * a.at<float>(1) - normal.at<float>(1) * a.at<float>(0));
            b = b / cv::norm(b);

            float fMaxDis2 = fMaxDis * fMaxDis;
            float fMinDis2 = fMinDis * fMinDis;
            for (float disi = fMinDis * cos(fMaxDegree); disi < fMaxDis; disi += mfResolution) {
                cv::Mat pos_c = Pos - disi * normal;
                if (disi >= fMinDis) {
                    int index_x = (pos_c.at<float>(0)) / mfResolution;
                    int index_y = (pos_c.at<float>(1)) / mfResolution;
                    int index_z = (pos_c.at<float>(2)) / mfResolution;
                    Eigen::Vector3i index_voxel(index_x, index_y, index_z);
                     {
                        if (!mCubeMap.count(index_voxel)) {
                            mnCubeNum++;
                            mCubeMap[index_voxel] = set<long unsigned int>();
                            mCubeMap[index_voxel].insert(pMP->mnId);
                        } else {
                            mCubeMap[index_voxel].insert(pMP->mnId);
                        }
                    }
                }

                float rmax = disi * tan(fMaxDegree);
                float disi2 = disi * disi;
                for (float ri = mfResolution; ri < rmax; ri += mfResolution) {
                    if (ri * ri + disi2 > fMaxDis2)
                        break;
                    if (ri * ri + disi2 < fMinDis2)
                        continue;
                    float th_step = atan(mfResolution / ri);
                    for (float th = 0.0; th < 6.2831853; th += th_step) {
                        float pos_x =
                                pos_c.at<float>(0) + ri * cos(th) * a.at<float>(0) + ri * sin(th) * b.at<float>(0);
                        float pos_y =
                                pos_c.at<float>(1) + ri * cos(th) * a.at<float>(1) + ri * sin(th) * b.at<float>(1);
                        float pos_z =
                                pos_c.at<float>(2) + ri * cos(th) * a.at<float>(2) + ri * sin(th) * b.at<float>(2);

                        {
                            int index_x = (pos_x) / mfResolution;
                            int index_y = (pos_y) / mfResolution;
                            int index_z = (pos_z) / mfResolution;
                            Eigen::Vector3i index_voxel(index_x, index_y, index_z);
                            {
                                if (!mCubeMap.count(index_voxel)) {
                                    mnCubeNum++;
                                    mCubeMap[index_voxel] = set<long unsigned int>();
                                    mCubeMap[index_voxel].insert(pMP->mnId);
                                } else {
                                    mCubeMap[index_voxel].insert(pMP->mnId);
                                }
                            }
                        }
                    }
                }
            }
            cout << "************** Processed: " << i << "/" << mspMapPoints.size() << "**************" << endl;
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double tt = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        cout << "using " << tt << " s" << endl;
    }

} //namespace ORB_SLAM
