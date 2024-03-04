/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{  
    // if(argc < 5)
    // {
    //     cerr << endl << "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;
    //     return 1;
    // }

    // Open webcam
    cout << "Opening webcam" << endl;
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        cerr << endl << "Failed to open camera" << endl;
        return 1;
    }

    // create a window to display the images from the webcam
    // cv::namedWindow("Webcam", cv::WINDOW_AUTOSIZE);

    // array to hold image
    cout << "Creating image array" << endl;
    cv::Mat frame;
    cv::Mat frame_gray;
    cv::Mat frame_gray_resized;
    
    // // display the frame until you press a key
    // while (1) {
    //     // capture the next frame from the webcam
    //     cap >> frame;
    //     // convert to grayscale
    //     cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    //     cout << "frame_gray.cols = " << frame_gray.cols << endl;
    //     cout << "frame_gray.rows = " << frame_gray.rows << endl;
    //
    //     // resize the image to 752x480
    //     cv::resize(frame_gray, frame_gray_resized, cv::Size(752, 480));
    //     cout << "frame_gray_resized.cols = " << frame_gray_resized.cols << endl;
    //     cout << "frame_gray_resized.rows = " << frame_gray_resized.rows << endl;
    //
    //     // show the image on the window
    //     cv::imshow("Webcam", frame_gray_resized);
    //
    //     // wait (10ms) for esc key to be pressed to stop
    //     if (cv::waitKey(10) == 27)
    //         break;
    // }
    // return 0;

    // Load all sequences:
    const int num_seq = 1;//(argc-3)/2;
    bool bFileName= 0;//(((argc-3) % 2) == 1);

    int seq = 0;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    // string strImagePath = string(argv[(2*seq)+3]) + "/mav0/cam0/data";
    // string strPathTimes = string(argv[(2*seq)+4]);
    string strImagePath = "/home/sebtheiler/Datasets/EuRoc/TEST/mav0/cam0/data";
    string strPathTimes = "/home/sebtheiler/Datasets/EuRoc/TEST/mav0/timestamps.txt";
    LoadImages(strImagePath, strPathTimes, vstrImageFilenames[seq], vTimestampsCam[seq]);
    cout << "LOADED!" << endl;
    nImages[seq] = vstrImageFilenames[seq].size();
    tot_images += nImages[seq];

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);


    int fps = 20;
    float dT = 1.f/fps;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    for (seq = 0; seq<num_seq; seq++)
    {

        // Main loop
        cv::Mat im;
        int proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {

            // Read image from file
            // cout << "Reading image " << vstrImageFilenames[seq][ni] << endl;
            // im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED); //,CV_LOAD_IMAGE_UNCHANGED);

            // Read image from webcam
            cout << "ni = " << ni << endl;
            cout << "seq = " << seq << endl;
            cout << "proccIm = " << proccIm << endl;
            cout << "Reading image from webcam" << endl;
            cap >> frame;
            // convert to grayscale
            cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
            cout << "frame_gray.cols = " << frame_gray.cols << endl;
            cout << "frame_gray.rows = " << frame_gray.rows << endl;

            // resize the image to 752x480
            cv::resize(frame_gray, frame_gray_resized, cv::Size(752, 480));

            // Copy data to ORB_SLAM3 image structure
            im = frame_gray_resized.clone();

            cout << "Processing image " << proccIm << " of sequence " << seq << endl;
            cout << "im.cols = " << im.cols << endl;
            cout << "im.rows = " << im.rows << endl;
            cout << "im.empty() = " << im.empty() << endl;
            cout << "imageScale = " << imageScale << endl;

            double tframe = vTimestampsCam[seq][ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

            if(imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
                cout "register time and compile with c11" << endl;
    #else
                std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
                cout "register time and not compile with c11" << endl;
    #endif
#endif
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
            }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            cout << "tframe = " << tframe << endl;
            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            //std::cout << "T: " << T << std::endl;
            //std::cout << "ttrack: " << ttrack << std::endl;

            if(ttrack<T) {
                //std::cout << "usleep: " << (dT-ttrack) << std::endl;
                usleep((T-ttrack)*1e6); // 1e6
            }
        }

        if(seq < num_seq - 1)
        {
            string kf_file_submap =  "./SubMaps/kf_SubMap_" + std::to_string(seq) + ".txt";
            string f_file_submap =  "./SubMaps/f_SubMap_" + std::to_string(seq) + ".txt";
            SLAM.SaveTrajectoryEuRoC(f_file_submap);
            SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file_submap);

            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }

    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // if (bFileName)
    // {
    //     const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
    //     const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
    //     SLAM.SaveTrajectoryEuRoC(f_file);
    //     SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    // }
    // else
    // {
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    // }

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t*1e-9);

        }
    }
}
