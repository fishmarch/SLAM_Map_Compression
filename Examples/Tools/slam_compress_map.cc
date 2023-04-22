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
//* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <malloc.h>

#include<opencv2/core/core.hpp>

#include"include/System.h"

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv) {
    if (argc != 6) {
        cerr << endl
             << "Usage: ./rgbd_tum_loadmap path_to_vocabulary path_to_settings path_to_map path_to_compressed_map method."
             << endl;
        return 1;
    }

    int CompressionMethodId = 0;
    stringstream ss(argv[5]);
    ss >> CompressionMethodId;
    cout << "Compression Method: " << CompressionMethodId << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true,true);
    SLAM.CloseImageViewer();
    SLAM.LoadMap(argv[3], argv[2]);

    cout << "Press Enter to continue!" << endl;
    getchar();
    switch (CompressionMethodId) {
        case 1:
            cout << "Map compression using CompressMapByGrid2D" << endl;
            SLAM.CompressMapByGrid2D();
            break;
        case 2:
            cout << "Map compression using CompressMapByGrid3D" << endl;
            SLAM.CompressMapByGrid3D();
            break;
        default:
            cerr << "Undefined method!!!" << endl;
    }

    SLAM.DeleteMPs();

    cout << "Press Enter to continue!" << endl;
    getchar();

    SLAM.SaveMap(argv[4]);

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps) {
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while (!fAssociation.eof()) {
        string s;
        getline(fAssociation, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
