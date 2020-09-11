#include <opencv2/opencv.hpp>
#include <calib_zed.hpp>
#include <iostream>
#include <vector>

using namespace std;

void seeMovie();

int main()
{
    //auto dist = getDistortionCoefficient();

    // cout << "distcoeff's size: " << dist.size() << endl;
    // for(int i=0 ; dist.size() ; i++)
    //     cout << dist[i] <<'\n';
    // cout << endl;

    return 0;
}

void seeMovie()
{
        int select_00[] = {}; // 4500 프레임 / 20 = 225장
        // 01, 02시퀀스는 완전 도로위라서 우리가 테스트하고자 하는 환경과는 맞지 않는 경향 있음.
    int select_03[] = {}; // 20개 단위, 800 프레임 / 20 = 40 장.
    int select_05[] = {}; // 2760 frame / 20 = 138 장
    int select_06[] = {}; // 1100 frame / 20 = 55 장

    cv::String root_dir = "/home/leecw/Datasets/kitti_gray_dataset/sequences/";
    cv::String path_00 = root_dir + "00/image_0/*.png";
    cv::String path_03 = root_dir + "03/image_0/*.png";
    cv::String path_05 = root_dir + "05/image_0/*.png";
    cv::String path_06 = root_dir + "06/image_0/*.png";

    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);

    vector< cv::String > files;
    cv::glob(path_03, files, false);

    for(size_t i=0; i<files.size(); ++i)
    {
        cv::Mat img = cv::imread(files[i]);
        if(img.empty())
        {
            cout << files[i] << " is invalid!\n";
            continue;
        }
        else
        {
            cv::imshow("image",  img);
            cv::waitKey(1000);
        }

        cout << img.size() << endl;

    }

    cv::destroyAllWindows();
}
